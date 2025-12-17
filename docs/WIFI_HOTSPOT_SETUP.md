# RSSAEM Robot WiFi Hotspot Setup

로봇이 인터넷 없이 독립적으로 작동할 수 있도록 WiFi 핫스팟 모드를 설정합니다.

## 개요

```
스마트폰/노트북 ──WiFi──> 로봇 (AP모드) ──> 웹 인터페이스
                         192.168.4.1
```

## 필요 패키지

```bash
sudo apt update
sudo apt install -y hostapd dnsmasq
```

## 설정 파일

### 1. NetworkManager 비활성화 (WiFi 인터페이스)

```bash
# WiFi 인터페이스 확인
ip link show | grep wl

# NetworkManager에서 WiFi 제외
sudo nano /etc/NetworkManager/NetworkManager.conf
```

추가:
```ini
[keyfile]
unmanaged-devices=interface-name:wlan0
```

### 2. 고정 IP 설정

```bash
sudo nano /etc/dhcpcd.conf
```

추가:
```
interface wlan0
static ip_address=192.168.4.1/24
nohook wpa_supplicant
```

### 3. hostapd 설정 (AP 모드)

```bash
sudo nano /etc/hostapd/hostapd.conf
```

```ini
interface=wlan0
driver=nl80211
ssid=RSSAEM_Robot
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=12345678
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```

hostapd 기본 설정 파일 지정:
```bash
sudo nano /etc/default/hostapd
```

```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

### 4. dnsmasq 설정 (DHCP)

```bash
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
sudo nano /etc/dnsmasq.conf
```

```ini
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
domain=robot.local
address=/robot.local/192.168.4.1
```

### 5. 서비스 활성화

```bash
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq
sudo reboot
```

## 접속 방법

1. 스마트폰/노트북에서 WiFi 검색
2. **RSSAEM_Robot** 네트워크 연결
3. 비밀번호: `12345678`
4. 브라우저에서 접속:
   - http://192.168.4.1/
   - http://robot.local/ (mDNS 지원 시)

## 모드 전환 스크립트

### AP 모드로 전환

```bash
#!/bin/bash
# /home/nvidia/ros2_ws/scripts/enable_ap_mode.sh

sudo systemctl stop NetworkManager
sudo systemctl start hostapd
sudo systemctl start dnsmasq
echo "AP Mode enabled: SSID=RSSAEM_Robot"
echo "Connect to: http://192.168.4.1/"
```

### 클라이언트 모드로 전환

```bash
#!/bin/bash
# /home/nvidia/ros2_ws/scripts/enable_client_mode.sh

sudo systemctl stop hostapd
sudo systemctl stop dnsmasq
sudo systemctl start NetworkManager
echo "Client mode enabled"
echo "Connect to existing WiFi network"
```

## 자동 시작 (부팅 시)

```bash
sudo nano /etc/rc.local
```

```bash
#!/bin/bash
# Start robot services
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
export LIDAR_MODEL=LDS-04
export RSSAEM_MODEL=robot

# Start ROS2 launch in background
ros2 launch robot_web web_interface.launch.py &

exit 0
```

```bash
sudo chmod +x /etc/rc.local
```

## 문제 해결

### hostapd 시작 실패
```bash
# 로그 확인
sudo journalctl -u hostapd -f

# WiFi 인터페이스 상태 확인
sudo rfkill list
sudo rfkill unblock wifi
```

### IP 충돌
```bash
# 다른 네트워크 대역 사용
# 192.168.4.x 대신 10.10.10.x 등
```

### mDNS 설정 (선택)
```bash
sudo apt install avahi-daemon
# robot.local 자동 해석
```

## 네트워크 구성도

```
┌─────────────────────────────────────────────────┐
│                  RSSAEM Robot                    │
│  ┌───────────────────────────────────────────┐  │
│  │  wlan0 (AP Mode)                          │  │
│  │  SSID: RSSAEM_Robot                       │  │
│  │  IP: 192.168.4.1                          │  │
│  └───────────────────────────────────────────┘  │
│                      │                          │
│  ┌───────────────────────────────────────────┐  │
│  │  Services                                 │  │
│  │  - nginx (80) → Web Interface             │  │
│  │  - rosbridge (9090) → WebSocket           │  │
│  │  - web_video_server (8080) → Camera       │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
                      │
                     WiFi
                      │
┌─────────────────────────────────────────────────┐
│  Client Device (Phone/Laptop)                   │
│  IP: 192.168.4.x (DHCP)                        │
│  Browser: http://192.168.4.1/                  │
└─────────────────────────────────────────────────┘
```

## 참고

- Jetson Orin Nano의 WiFi 모듈이 AP 모드를 지원해야 함
- 일부 WiFi 어댑터는 AP 모드 미지원
- USB WiFi 동글 사용 시 nl80211 드라이버 필요
