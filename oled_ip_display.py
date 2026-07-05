import smbus2
import time
import socket
import logging
import subprocess
from datetime import datetime, timedelta

# Configuration 
logging.basicConfig(filename='/var/log/oled_ip_display.log', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

# Configuration parameters
CONFIG = {
    'OLED_ADDRESS': 0x3C,
    'I2C_BUS': 1,
    'IP_CHECK_TIMEOUT': 120,
    'IP_CHECK_INTERVAL': 5,
    'DISPLAY_UPDATE_INTERVAL': 30,
    'SERVICE_CHECK_INTERVAL': 10,  # 检查服务状态的间隔(秒)
    'SERVICE_NAME': 'main-server.service'  # 要监控的服务名称
}

# 创建 I2C 总线实例
bus = smbus2.SMBus(CONFIG['I2C_BUS'])

def oled_command(cmd):
    bus.write_byte_data(CONFIG['OLED_ADDRESS'], 0x00, cmd)

def oled_data(data):
    bus.write_byte_data(CONFIG['OLED_ADDRESS'], 0x40, data)

def init_oled():
    # 初始化序列（保持不变）
    oled_command(0xAE)  # 关闭显示
    oled_command(0xD5)  # 设置显示时钟分频比/振荡器频率
    oled_command(0x80)
    oled_command(0xA8)  # 设置多路复用率
    oled_command(0x1F)  # 0x1F for 128*32, 0x3F for 128*64
    oled_command(0xD3)  # 设置显示偏移
    oled_command(0x00)
    oled_command(0x40)  # 设置显示开始行
    oled_command(0x8D)  # 充电泵设置
    oled_command(0x14)
    oled_command(0x20)  # 设置内存地址模式
    oled_command(0x00)
    oled_command(0xA1)  # 设置段重新映射
    oled_command(0xC8)  # 设置 COM 输出扫描方向
    oled_command(0xDA)  # 设置 COM 引脚硬件配置
    oled_command(0x02)  # 0x02 for 128*32, 0x12 for 128*64
    oled_command(0x81)  # 设置对比度控制
    oled_command(0xCF)
    oled_command(0xD9)  # 设置预充电周期
    oled_command(0xF1)
    oled_command(0xDB)  # 设置 VCOMH 取消选择级别
    oled_command(0x30)
    oled_command(0xA4)  # 整个显示打开
    oled_command(0xA6)  # 设置正常显示
    oled_command(0xAF)  # 打开显示

def clear_display():
    for page in range(4):  # 4 pages for 128*32
        oled_command(0xB0 + page)  # 设置页地址
        oled_command(0x00)         # 设置列地址低位
        oled_command(0x10)         # 设置列地址高位
        for i in range(128):
            oled_data(0x00)

def display_text(text, row):
    oled_command(0xB0 + row)  # 设置页地址
    oled_command(0x00)        # 设置列地址低位
    oled_command(0x10)        # 设置列地址高位
    for char in text:
        if char in font:
            for col in font[char]:
                oled_data(col)
        else:
            # 如果字符不在字体中，显示空白
            for _ in range(5):
                oled_data(0x00)
        oled_data(0x00)  # 字符间距

def get_ip_address():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2)  # 设置超时时间为2秒
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            return ip
        except (socket.timeout, socket.error):
            return None
        finally:
            s.close()
    except Exception as e:
        logging.error(f"Socket creation error: {e}")
        return None

def check_service_status(service_name):
    """
    检查系统服务状态
    返回: (is_active, status_message)
    """
    try:
        # 检查服务是否处于活动状态
        result = subprocess.run(['systemctl', 'is-active', service_name], 
                              capture_output=True, text=True)
        is_active = result.stdout.strip() == 'active'
        
        # 获取详细状态
        status = subprocess.run(['systemctl', 'status', service_name], 
                              capture_output=True, text=True)
        
        if is_active:
            message = "Running"
            logging.info(f"Service {service_name} is running")
        else:
            message = "Stopped"
            logging.warning(f"Service {service_name} is not running: {status.stdout}")
        
        return is_active, message
        
    except Exception as e:
        logging.error(f"Error checking service status: {e}")
        return False, "Error"

# 扩展的 5x8 字体
font = {
    '0': [0x3E, 0x51, 0x49, 0x45, 0x3E],
    '1': [0x00, 0x42, 0x7F, 0x40, 0x00],
    '2': [0x42, 0x61, 0x51, 0x49, 0x46],
    '3': [0x21, 0x41, 0x45, 0x4B, 0x31],
    '4': [0x18, 0x14, 0x12, 0x7F, 0x10],
    '5': [0x27, 0x45, 0x45, 0x45, 0x39],
    '6': [0x3C, 0x4A, 0x49, 0x49, 0x30],
    '7': [0x01, 0x71, 0x09, 0x05, 0x03],
    '8': [0x36, 0x49, 0x49, 0x49, 0x36],
    '9': [0x06, 0x49, 0x49, 0x29, 0x1E],
    '.': [0x00, 0x60, 0x60, 0x00, 0x00],
    ':': [0x00, 0x36, 0x36, 0x00, 0x00],
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00],
    'A': [0x7E, 0x11, 0x11, 0x11, 0x7E],
    'B': [0x7F, 0x49, 0x49, 0x49, 0x36],
    'C': [0x3E, 0x41, 0x41, 0x41, 0x22],
    'D': [0x7F, 0x41, 0x41, 0x22, 0x1C],
    'E': [0x7F, 0x49, 0x49, 0x49, 0x41],
    'F': [0x7F, 0x09, 0x09, 0x09, 0x01],
    'G': [0x3E, 0x41, 0x49, 0x49, 0x7A],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'I': [0x00, 0x41, 0x7F, 0x41, 0x00],
    'J': [0x20, 0x40, 0x41, 0x3F, 0x01],
    'K': [0x7F, 0x08, 0x14, 0x22, 0x41],
    'L': [0x7F, 0x40, 0x40, 0x40, 0x40],
    'M': [0x7F, 0x02, 0x0C, 0x02, 0x7F],
    'N': [0x7F, 0x04, 0x08, 0x10, 0x7F],
    'O': [0x3E, 0x41, 0x41, 0x41, 0x3E],
    'P': [0x7F, 0x09, 0x09, 0x09, 0x06],
    'Q': [0x3E, 0x41, 0x51, 0x21, 0x5E],
    'R': [0x7F, 0x09, 0x19, 0x29, 0x46],
    'S': [0x46, 0x49, 0x49, 0x49, 0x31],
    'T': [0x01, 0x01, 0x7F, 0x01, 0x01],
    'U': [0x3F, 0x40, 0x40, 0x40, 0x3F],
    'V': [0x1F, 0x20, 0x40, 0x20, 0x1F],
    'W': [0x3F, 0x40, 0x38, 0x40, 0x3F],
    'X': [0x63, 0x14, 0x08, 0x14, 0x63],
    'Y': [0x07, 0x08, 0x70, 0x08, 0x07],
    'Z': [0x61, 0x51, 0x49, 0x45, 0x43],
    'a': [0x20, 0x54, 0x54, 0x54, 0x78],
    'b': [0x7F, 0x48, 0x44, 0x44, 0x38],
    'c': [0x38, 0x44, 0x44, 0x44, 0x20],
    'd': [0x38, 0x44, 0x44, 0x48, 0x7F],
    'e': [0x38, 0x54, 0x54, 0x54, 0x18],
    'f': [0x08, 0x7E, 0x09, 0x01, 0x02],
    'g': [0x0C, 0x52, 0x52, 0x52, 0x3E],
    'h': [0x7F, 0x08, 0x04, 0x04, 0x78],
    'i': [0x00, 0x44, 0x7D, 0x40, 0x00],
    'j': [0x20, 0x40, 0x44, 0x3D, 0x00],
    'k': [0x7F, 0x10, 0x28, 0x44, 0x00],
    'l': [0x00, 0x41, 0x7F, 0x40, 0x00],
    'm': [0x7C, 0x04, 0x18, 0x04, 0x78],
    'n': [0x7C, 0x08, 0x04, 0x04, 0x78],
    'o': [0x38, 0x44, 0x44, 0x44, 0x38],
    'p': [0x7C, 0x14, 0x14, 0x14, 0x08],
    'q': [0x08, 0x14, 0x14, 0x18, 0x7C],
    'r': [0x7C, 0x08, 0x04, 0x04, 0x08],
    's': [0x48, 0x54, 0x54, 0x54, 0x20],
    't': [0x04, 0x3F, 0x44, 0x40, 0x20],
    'u': [0x3C, 0x40, 0x40, 0x20, 0x7C],
    'v': [0x1C, 0x20, 0x40, 0x20, 0x1C],
    'w': [0x3C, 0x40, 0x30, 0x40, 0x3C],
    'x': [0x44, 0x28, 0x10, 0x28, 0x44],
    'y': [0x0C, 0x50, 0x50, 0x50, 0x3C],
    'z': [0x44, 0x64, 0x54, 0x4C, 0x44]
}

def get_wlan_ssid():
    ssid = "NONE"
    try:
        result = subprocess.run(['iwgetid', '-r'], capture_output=True, text=True)
        ssid   = result.stdout.strip()
    except Exception as e:
	    print(f"[ERROR] Something Went Wrong in get_wlan_ssid {e}")
    return ssid

def wait_for_ip():
    start_time = datetime.now()
    while (datetime.now() - start_time).total_seconds() < CONFIG['IP_CHECK_TIMEOUT']:
        ip = get_ip_address()
        if ip:
            logging.info(f"IP found: {ip}")
            return ip
        
        # 显示等待消息
        clear_display()
        display_text("Getting IP", 0)
        display_text("Please wait", 2)
        
        time.sleep(CONFIG['IP_CHECK_INTERVAL'])
    
    logging.warning("IP check timeout")
    return "IP not found"

def format_display_line(label, value):
    """
    格式化显示行，确保标签和值在同一行
    """
    return f"{label}{value}"

def main():
    try:
        init_oled()
        clear_display()
        
        # 初始化时间记录
        last_ip_check = datetime.now()
        last_service_check = datetime.now()
        
        # 初始状态
        ip = wait_for_ip()
        service_status = "Checking"
        
        while True:
            current_time = datetime.now()
            
            # 检查IP
            if (current_time - last_ip_check).total_seconds() >= CONFIG['DISPLAY_UPDATE_INTERVAL']:
                new_ip = get_ip_address()
                if new_ip:
                    ip = new_ip
                last_ip_check = current_time
            
            # 检查服务状态
            if (current_time - last_service_check).total_seconds() >= CONFIG['SERVICE_CHECK_INTERVAL']:
                _, service_status = check_service_status(CONFIG['SERVICE_NAME'])
                last_service_check = current_time
            
            # 更新显示
            clear_display()
            
            # 在两行上显示IP和服务状态
            display_text(format_display_line("SSID: ", get_wlan_ssid()),0)
            display_text(format_display_line("IP:", ip if ip else "No IP"), 1)
            display_text(format_display_line("Server:", service_status), 3)
            
            time.sleep(10)  # 降低CPU使用率
            
    except Exception as e:
        logging.error(f"An error occurred: {e}")
        if 'I/O error' in str(e):
            logging.error("Check your I2C connection and address")
        raise

if __name__ == "__main__":
    main()