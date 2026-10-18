import requests

BLUEOS_IP = "192.168.42.1"

# MAVLink2REST 経由で高度取得
url = f"http://{BLUEOS_IP}/mavlink2rest/mavlink/vehicles/1/components/1/messages/GLOBAL_POSITION_INT"
response = requests.get(url)

if response.status_code == 200:
    data = response.json()
    alt = data["message"]["relative_alt"] / 1000
    print(f"高度: {alt:.1f} m")
else:
    print(f"エラー: {response.status_code}")