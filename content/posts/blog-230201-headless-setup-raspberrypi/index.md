---
title: "Headlessly setup raspberry pi without connecting to monitor and keyboard"
date: 2023-02-01T13:53:39+08:00
ShowToc: true
TocOpen: true
tags: ["Raspberry Pi"]
author: ["Kevin Lee"]
draft: true
---

### 前言
對於樹莓派玩家而言，用ssh登入在主機端操作，是常見的做法。
但是為了要達成此目的，在一開始還是得接上螢幕、鍵盤，進行WiFi連接等設定，十分不便。
本文目的即在解決此問題。

### 測試環境
- Host: MacBook
- Raspberry Pi版本: 4B/8G

測試雖在Mac環境，但是在Windows下原則上大同小異，簡單可分為以下步驟

### 1. 下載來源image檔

### 2. 燒錄至SD Card
燒錄方式一般有以下幾種，基本上選擇自己習慣的即可
- balenaEtcher
- Raspberry Pi Imager
- commmand line
  - 下圖整理了燒錄/掛載/卸載/磁區檢查等常見操作
    {{< figure src="image1.png" >}}

### 3. network-config & user-data
燒錄完成後，將SD Card重新插拔，檢查其磁區與掛載，可使用以下兩個指令
{{< details `diskutil list` >}}
```bash
~ ❯ diskutil list
/dev/disk0 (internal, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      GUID_partition_scheme                        *2.0 TB     disk0
   1:                        EFI ⁨EFI⁩                     314.6 MB   disk0s1
   2:                 Apple_APFS ⁨Container disk1⁩         2.0 TB     disk0s2

/dev/disk1 (synthesized):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      APFS Container Scheme -                      +2.0 TB     disk1
                                 Physical Store disk0s2
   1:                APFS Volume ⁨Macintosh HD⁩            15.4 GB    disk1s1
   2:              APFS Snapshot ⁨com.apple.os.update-...⁩ 15.4 GB    disk1s1s1
   3:                APFS Volume ⁨Macintosh HD - Data⁩     1.4 TB     disk1s2
   4:                APFS Volume ⁨Preboot⁩                 372.4 MB   disk1s3
   5:                APFS Volume ⁨Recovery⁩                1.1 GB     disk1s4
   6:                APFS Volume ⁨VM⁩                      5.4 GB     disk1s5

/dev/disk2 (external, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:     FDisk_partition_scheme                        *31.9 GB    disk2
   1:             Windows_FAT_32 ⁨system-boot⁩             268.4 MB   disk2s1
   2:                      Linux ⁨⁩                        31.7 GB    disk2s2
```
{{< /details >}}
{{< details `dh -h` >}}
```bash
~ ❯ df -h
Filesystem       Size   Used  Avail Capacity iused      ifree %iused  Mounted on
/dev/disk1s1s1  1.8Ti   14Gi  537Gi     3%  502068 4291231305    0%   /
devfs           397Ki  397Ki    0Bi   100%    1374          0  100%   /dev
/dev/disk1s5    1.8Ti  5.0Gi  537Gi     1%       5 5633182040    0%   /System/Volumes/VM
/dev/disk1s3    1.8Ti  355Mi  537Gi     1%    1949 5633182040    0%   /System/Volumes/Preboot
/dev/disk1s6    1.8Ti  5.6Mi  537Gi     1%      33 5633182040    0%   /System/Volumes/Update
/dev/disk1s2    1.8Ti  1.3Ti  537Gi    71% 7235966 5633182040    0%   /System/Volumes/Data
map auto_home     0Bi    0Bi    0Bi   100%       0          0  100%   /System/Volumes/Data/home
/dev/disk2s1    252Mi   61Mi  191Mi    25%       0          0  100%   /Volumes/system-boot
/dev/disk1s1    1.8Ti   14Gi  537Gi     3%  502070 4291146493    0%   /System/Volumes/Update/mnt1
```
{{< /details >}}

發現其第一個磁區，已掛載在/Volumes/system-boot下，檢查其檔案，會發現有兩個重要的檔案network-config和user-data。
接下來，我們便要針對此兩個檔案進行修改。
{{< details `/Volumes/system-boot檔案結構` >}}
```bash
/Volumes/system-boot ❯ tree -L 1
.
├── README
├── bcm2710-rpi-2-b.dtb
├── bcm2710-rpi-3-b-plus.dtb
├── bcm2710-rpi-3-b.dtb
├── bcm2710-rpi-cm3.dtb
├── bcm2710-rpi-zero-2.dtb
├── bcm2711-rpi-4-b.dtb
├── bcm2711-rpi-400.dtb
├── bcm2711-rpi-cm4.dtb
├── bcm2837-rpi-3-a-plus.dtb
├── bcm2837-rpi-3-b-plus.dtb
├── bcm2837-rpi-3-b.dtb
├── bcm2837-rpi-cm3-io3.dtb
├── boot.scr
├── bootcode.bin
├── cmdline.txt
├── config.txt
├── fixup.dat
├── fixup4.dat
├── fixup4cd.dat
├── fixup4db.dat
├── fixup4x.dat
├── fixup_cd.dat
├── fixup_db.dat
├── fixup_x.dat
├── initrd.img
├── meta-data
├── network-config
├── overlays
├── start.elf
├── start4.elf
├── start4cd.elf
├── start4db.elf
├── start4x.elf
├── start_cd.elf
├── start_db.elf
├── start_x.elf
├── syscfg.txt
├── uboot_rpi_3.bin
├── uboot_rpi_4.bin
├── uboot_rpi_arm64.bin
├── user-data
├── usercfg.txt
└── vmlinuz
```
{{< /details >}}

* 修改 /boot/firmware/network-config
```
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        your_ssid:
          password: "your_password"
```
* 修改 /boot/firmware/user-data

此檔案可直接設定user和hostname，hostname和avahi-daemon搭配，方邊後續登入。
其為ubuntu提供的cloud-init的一部分，細節可參考[cloud-init doc](https://cloudinit.readthedocs.io/en/latest/reference/examples.html#)
```
#cloud-config

## Set the system's hostname.
hostname: your_host_name

# Enable password authentication with the SSH daemon
ssh_pwauth: true

## Add users and groups to the system, and import keys with the ssh-import-id
users:
- name: pi
  sudo: ALL=(ALL) NOPASSWD:ALL
  groups: users,adm,dialout,audio,netdev,video,plugdev,cdrom,games,input,gpio,spi,i2c,render,sudo
  lock_passwd: false
  shell: /bin/bash
  passwd: raspberry

## Run arbitrary commands at rc.local like time
runcmd:
 - [ ls, -l, / ]
 - [ sh, -xc, "echo $(date) ': hello world!'" ]

power_state:
  mode: reboot
```

### 4. 第一次登入
完成上一步後，重啟raspberry pi，使用以下指令，等其連上區網並確定其IP。這個過程要2~5分鐘，有時甚至要再次重啟。
```bash
nmap -sn 172.20.10.1/24
```
找出其IP (ex: 172.20.10.3) 後，使用以下指令ssh登入
```bash
ssh pi@172.20.10.3
```

### 5. 使用.local網域登入
* 利用avahi-daemon，可以使用.local網域直接登入而不需使用IP

_in raspberry pi_
```bash
sudo apt update
sudo apt install avahi-daemon
```
_in host_
```bash
ping your_host_name.local
ssh pi@your_host_name.local
```
### 6. 免密碼ssh登入
* 使用ssh-copy-id，可以免去每次ssh使用密碼登入
```bash
# 這邊需準備一對金鑰，假設其路徑為~/.ssh/id_rsa.pub
ssh-copy-id -i ~/.ssh/id_rsa.pub pi@your_host_name.local
```

### 後記
此流程，針對以下image皆可順利設定成功，更多img檔有待進一步測試，以測試此法的有效性
- focal-preinstalled-server-arm64+raspi.img
- ubuntu-20.04.5-preinstalled-server-arm64+raspi.img

### Reference
1. https://www.youtube.com/watch?v=IW_6EEAyXGs
2. [Install Ubuntu 20.04 on Raspberry Pi 4 (without monitor)](https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/)
3. [How to Set up WiFi on Your Raspberry Pi Without a Monitor (Headless)](https://howchoo.com/g/ndy1zte2yjn/how-to-set-up-wifi-on-your-raspberry-pi-without-ethernet)
