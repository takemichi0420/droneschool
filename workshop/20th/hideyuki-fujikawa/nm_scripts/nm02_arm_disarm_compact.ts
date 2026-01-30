#!/usr/bin/env -S npx ts-node
import nodeMavlink from 'node-mavlink'
const { MavTCP, minimal, common, ardupilotmega, reserialize, MavLinkPacket, MavLinkPacketRegistry } = nodeMavlink

const REGISTRY: MavLinkPacketRegistry = { ...minimal.REGISTRY, ...common.REGISTRY, ...ardupilotmega.REGISTRY }

async function main() {
    const port = new MavTCP()

    const shutdown = async (signal: NodeJS.Signals) => {
        console.log(`\n${signal} received, closing connection...`)
        await port.close().catch(() => {})
        process.exit(0)
    }

    ;['SIGINT', 'SIGTERM'].forEach(sig => process.once(sig, shutdown))

    // start the communication
    const { ip } = await port.start('192.168.3.38', 5762)
    console.log(`Connected to: ${ip}`)

    // log incoming messages
    port.on('data', (packet: MavLinkPacket) => {
        const clazz = REGISTRY[packet.header.msgid]
        if (!clazz) {
            console.log('<UNKNOWN>', packet.debug())
            return
        }
        const data = packet.protocol.data(packet.payload, clazz)
        const prefix = packet.header.msgid === common.CommandAck.MSG_ID ? 'ACKNOWLEDGED>' : ''
        console.log(packet.debug())
        console.log(prefix ? `${prefix} ${data}` : data)
    })

    // set message interval for GLOBAL_POSITION_INT to 10Hz
    const command = new common.SetMessageIntervalCommand()
    command.messageId = common.GlobalPositionInt.MSG_ID
    command.interval = 100_000 // 10Hz

    const armedCommand = new common.ComponentArmDisarmCommand()
    armedCommand.arm = 1
    armedCommand.force = 0

    await port.send(command)

    const { header, data } = reserialize(command)
    console.log(`Packet (proto: MAV_V2, sysid: ${header.sysid}, compid: ${header.compid}, msgid: ${header.msgid}, seq: ${header.seq}, plen: ${header.payloadLength})`)
    console.log('SENT>', data)

    await new Promise<void>(() => { })
}

main()
