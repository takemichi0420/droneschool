#!/usr/bin/env -S npx ts-node
import nodeMavlink from 'node-mavlink'

const { MavEsp8266, minimal, common, ardupilotmega, MavLinkPacket, MavLinkPacketRegistry, sleep } = nodeMavlink

const REGISTRY: MavLinkPacketRegistry = {
  ...minimal.REGISTRY,
  ...common.REGISTRY,
  ...ardupilotmega.REGISTRY,
}

type DecodedMessage<T> = {
  packet: MavLinkPacket
  data: T
}

interface ConnectionOptions {
  host: string
  sendPort: number
  listenPort: number
  targetSystem: number
  targetComponent: number
  heartbeatTimeoutMs: number
  ackTimeoutMs: number
  stateTimeoutMs: number
  disarmDelayMs: number
  forceParam: number
}

const numberOr = (value: string | undefined, fallback: number): number => {
  if (!value) return fallback
  const parsed = Number(value)
  return Number.isFinite(parsed) ? parsed : fallback
}

const defaultOptions: ConnectionOptions = {
  host: process.env.MAVLINK_REMOTE_HOST ?? '127.0.0.1',
  sendPort: numberOr(process.env.MAVLINK_REMOTE_PORT, 14551),
  listenPort: numberOr(process.env.MAVLINK_LOCAL_PORT, 14550),
  targetSystem: numberOr(process.env.MAVLINK_TARGET_SYSTEM, 1),
  targetComponent: numberOr(process.env.MAVLINK_TARGET_COMPONENT, 1),
  heartbeatTimeoutMs: numberOr(process.env.MAVLINK_HEARTBEAT_TIMEOUT, 10_000),
  ackTimeoutMs: numberOr(process.env.MAVLINK_ACK_TIMEOUT, 5_000),
  stateTimeoutMs: numberOr(process.env.MAVLINK_STATE_TIMEOUT, 10_000),
  disarmDelayMs: numberOr(process.env.MAVLINK_DISARM_DELAY, 5_000),
  forceParam: numberOr(process.env.MAVLINK_FORCE_PARAM, 0),
}

const FLAG_MAP: Record<string, keyof ConnectionOptions> = {
  host: 'host',
  'remote-port': 'sendPort',
  'local-port': 'listenPort',
  'target-system': 'targetSystem',
  'target-component': 'targetComponent',
  'heartbeat-timeout': 'heartbeatTimeoutMs',
  'ack-timeout': 'ackTimeoutMs',
  'state-timeout': 'stateTimeoutMs',
  'disarm-delay': 'disarmDelayMs',
  'force-param': 'forceParam',
}

const parseOptions = (): ConnectionOptions => {
  const options: ConnectionOptions = { ...defaultOptions }
  for (const arg of process.argv.slice(2)) {
    if (!arg.startsWith('--')) continue
    const [rawFlag, rawValue] = arg.slice(2).split('=')
    const flag = FLAG_MAP[rawFlag]
    if (!flag) continue
    if (typeof options[flag] === 'number') {
      options[flag] = numberOr(rawValue, options[flag] as number) as any
    } else if (typeof rawValue === 'string' && rawValue.length > 0) {
      options[flag] = rawValue as any
    }
  }
  return options
}

const waitForMessage = <T>(port: InstanceType<typeof MavEsp8266>, msgId: number, predicate: (decoded: DecodedMessage<T>) => boolean = () => true, timeoutMs = 10_000): Promise<DecodedMessage<T>> => {
  return new Promise((resolve, reject) => {
    const handler = (packet: MavLinkPacket) => {
      if (packet.header.msgid !== msgId) return
      const clazz = REGISTRY[msgId]
      if (!clazz) return
      const decoded: DecodedMessage<T> = {
        packet,
        data: packet.protocol.data(packet.payload, clazz) as T,
      }
      if (!predicate(decoded)) return
      cleanup()
      resolve(decoded)
    }

    const timeout = setTimeout(() => {
      cleanup()
      reject(new Error(`Timed out waiting for msgId ${msgId}`))
    }, timeoutMs)

    const cleanup = () => {
      clearTimeout(timeout)
      port.off('data', handler)
    }

    port.on('data', handler)
  })
}

const waitForArmState = async (port: InstanceType<typeof MavEsp8266>, armed: boolean, timeoutMs: number) => {
  const predicate = ({ data }: DecodedMessage<InstanceType<typeof minimal.Heartbeat>>) => {
    const isArmed = (data.baseMode & common.MavModeFlag.SAFETY_ARMED) === common.MavModeFlag.SAFETY_ARMED
    return isArmed === armed
  }
  await waitForMessage(port, common.Heartbeat.MSG_ID, predicate, timeoutMs)
}

const sendArmCommand = async (port: InstanceType<typeof MavEsp8266>, options: ConnectionOptions, arm: boolean) => {
  const command = new common.ComponentArmDisarmCommand(options.targetSystem, options.targetComponent)
  command.arm = arm ? 1 : 0
  command.force = arm ? options.forceParam : 0

  await port.send(command)

  const ack = await waitForMessage<InstanceType<typeof common.CommandAck>>(port, common.CommandAck.MSG_ID, ({ data }) => data.command === common.MavCmd.COMPONENT_ARM_DISARM, options.ackTimeoutMs)
  if (ack.data.result !== common.MavResult.ACCEPTED) {
    throw new Error(`Arm/disarm command rejected with result ${ack.data.result}`)
  }
}

async function main() {
  const options = parseOptions()
  const port = new MavEsp8266()

  const shutdown = async (signal: NodeJS.Signals) => {
    console.log(`\n${signal} received, closing connection...`)
    try {
      await port.close()
    } catch {
      // ignore errors when shutting down
    }
    process.exit(0)
  }
  ;['SIGINT', 'SIGTERM'].forEach(sig => process.once(sig, shutdown))

  console.log(`Listening on UDP ${options.listenPort}, sending to ${options.host}:${options.sendPort}`)
  await port.start(options.listenPort, options.sendPort, options.host)

  console.log('Waiting for heartbeat...')
  const heartbeat = await waitForMessage<InstanceType<typeof minimal.Heartbeat>>(port, common.Heartbeat.MSG_ID, undefined, options.heartbeatTimeoutMs)
  console.log(`Heartbeat received from system ${heartbeat.packet.header.sysid} component ${heartbeat.packet.header.compid}`)

  console.log('Sending arm command...')
  await sendArmCommand(port, options, true)
  await waitForArmState(port, true, options.stateTimeoutMs)
  console.log('ARMED')

  console.log(`Holding for ${(options.disarmDelayMs / 1000).toFixed(1)} seconds before disarm...`)
  await sleep(options.disarmDelayMs)

  console.log('Sending disarm command...')
  await sendArmCommand(port, options, false)
  await waitForArmState(port, false, options.stateTimeoutMs)
  console.log('DISARMED')

  await port.close()
}

main().catch(error => {
  console.error('Failed to arm/disarm via MAVLink:', error)
  process.exit(1)
})
