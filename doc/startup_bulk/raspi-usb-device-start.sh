#!/bin/bash

# 检查是否以root权限运行
if [ "$(id -u)" != "0" ]; then
   echo "此脚本需要root权限运行" 
   exit 1
fi

# 检查需要的目录是否存在
if [ ! -d "/sys/kernel/config/usb_gadget/" ]; then
    echo "USB gadget配置目录不存在，请确认系统支持USB gadget功能"
    echo "尝试加载相关模块..."
    modprobe libcomposite
    if [ ! -d "/sys/kernel/config/usb_gadget/" ]; then
        echo "加载模块后仍然无法找到USB gadget配置目录，脚本退出"
        exit 1
    fi
fi

cd /sys/kernel/config/usb_gadget/
mkdir -p pi4

cd pi4
echo 0x0955 > idVendor # Linux Foundation
echo 0x7020 > idProduct # Multifunction Composite Gadget
echo 0x0001 > bcdDevice # v1.0.0

echo 0x0200 > bcdUSB # USB2

echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
echo "abcdefg1234567890" > strings/0x409/serialnumber
echo "raspberry" > strings/0x409/manufacturer
echo "PI4" > strings/0x409/product

cfg=configs/c.1
mkdir -p "${cfg}"
echo 0x80 > ${cfg}/bmAttributes
echo 250 > ${cfg}/MaxPower

cfg_str=""

# 自动检测UDC设备
if [ -z "$(ls /sys/class/udc 2>/dev/null)" ]; then
    echo "没有找到UDC设备，可能需要加载相关驱动"
    exit 1
fi
udc_dev=$(ls /sys/class/udc | head -n 1)
echo "使用UDC设备: $udc_dev"

# The IP address shared by all USB network interfaces created by this script.
net_ip=192.168.55.1
# The associated netmask.
net_mask=255.255.255.0

# Note: RNDIS must be the first function in the configuration, or Windows'
# RNDIS support will not operate correctly.
enable_rndis=1
if [ ${enable_rndis} -eq 1 ]; then
    cfg_str="${cfg_str}+RNDIS"
    func=functions/rndis.usb0
    mkdir -p "${func}"
    ln -sf "${func}" "${cfg}"

    # Informs Windows that this device is compatible with the built-in RNDIS
    # driver. This allows automatic driver installation without any need for
    # a .inf file or manual driver selection.
    echo 1 > os_desc/use
    echo 0xcd > os_desc/b_vendor_code
    echo MSFT100 > os_desc/qw_sign
    echo RNDIS > "${func}/os_desc/interface.rndis/compatible_id"
    echo 5162001 > "${func}/os_desc/interface.rndis/sub_compatible_id"
    ln -sf "${cfg}" os_desc
fi

enable_bulk=1
if [ ${enable_bulk} -eq 1 ]; then
    mkdir -p /dev/usb-ffs
    
    cfg_str="${cfg_str}+BULK1"
    mkdir -p /dev/usb-ffs/bulk1
    func=functions/ffs.bulk1
    mkdir -p "${func}"
    ln -sf "${func}" "${cfg}"
    mount -o mode=0777 -o uid=2000 -o gid=2000 -t functionfs bulk1 /dev/usb-ffs/bulk1
    
    # 检查startup_bulk文件是否存在并可执行
    if [ ! -x "/home/qxx/cypress_codespace/Payload-SDK-Annotation/doc/startup_bulk/startup_bulk" ]; then
        echo "无法找到可执行文件: /home/qxx/cypress_codespace/Payload-SDK-Annotation/doc/startup_bulk/startup_bulk"
        echo "请确认文件存在并有执行权限"
        exit 1
    fi
    
    /home/qxx/cypress_codespace/Payload-SDK-Annotation/doc/startup_bulk/startup_bulk /dev/usb-ffs/bulk1 &
    sleep 3
    
    cfg_str="${cfg_str}+BULK2"
    mkdir -p /dev/usb-ffs/bulk2
    func=functions/ffs.bulk2
    mkdir -p "${func}"
    ln -sf "${func}" "${cfg}"
    mount -o mode=0777 -o uid=2000 -o gid=2000 -t functionfs bulk2 /dev/usb-ffs/bulk2
    /home/qxx/cypress_codespace/Payload-SDK-Annotation/doc/startup_bulk/startup_bulk /dev/usb-ffs/bulk2 &
    sleep 3
fi

mkdir -p "${cfg}/strings/0x409"
echo "${cfg_str:1}" > "${cfg}/strings/0x409/configuration"

udevadm settle -t 5 || :
echo "${udc_dev}" > UDC 2>/dev/null || { echo "写入UDC失败，设备可能已被占用"; }

# 检查网络命令是否存在
if ! command -v brctl &> /dev/null; then
    echo "找不到brctl命令，请安装bridge-utils"
    exit 1
fi

if ! command -v ifconfig &> /dev/null; then
    echo "找不到ifconfig命令，请安装net-tools"
    exit 1
fi

# 创建并配置网桥
/sbin/brctl addbr pi4br0 || echo "创建网桥失败"
/sbin/ifconfig pi4br0 ${net_ip} netmask ${net_mask} up || echo "配置网桥失败"

if [ ${enable_rndis} -eq 1 ]; then
    # 等待usb0接口出现
    echo "等待usb0接口出现..."
    for i in {1..10}; do
        if ifconfig -a | grep -q usb0; then
            echo "usb0接口已找到"
            /sbin/brctl addif pi4br0 usb0 || echo "添加接口到网桥失败"
            /sbin/ifconfig usb0 down || echo "关闭usb0失败"
            /sbin/ifconfig usb0 up || echo "启动usb0失败"
            break
        else
            echo "等待usb0接口出现，尝试 $i/10..."
            sleep 1
        fi
    done
    
    if ! ifconfig -a | grep -q usb0; then
        echo "usb0接口未出现，网络可能无法正常工作"
    fi
fi

echo "脚本执行完成"
exit 0