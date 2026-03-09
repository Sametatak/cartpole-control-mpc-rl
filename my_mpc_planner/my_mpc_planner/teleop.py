import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import sys, select, termios, tty

msg = """
Cart-Pole YUMUŞAK Klavye Kontrolü (TOPIC)
-----------------------------------------
Tuşa basılı tuttuğunda kuvvet kademeli artar,
Bıraktığınızda kademeli olarak sıfırlanır.

A / SOL OK : Sola İvmelen (Maks -300N)
D / SAĞ OK : Sağa İvmelen (Maks +300N)
BOŞLUK     : Ani Fren (0N)

CTRL+C ile çıkış yap.
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # Timeout 0.05 saniye (Saniyede 20 kez döngü çalışır)
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, sys.stdin.fileno(), settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    
    node = rclpy.create_node('teleop_cart_node')
    publisher = node.create_publisher(Wrench, '/cart_force', 10)

    print(msg)
    
    # Kademeli Kontrol Parametreleri
    MAX_FORCE = 50.0     # Çıkılabilecek maksimum kuvvet
    FORCE_STEP = 2.0     # Her döngüde artacak/azalacak kuvvet miktarı
    
    current_force = 0.0   # Arabaya o an uygulanan gerçek kuvvet
    target_force = 0.0    # Senin tuşla gitmek istediğin hedef kuvvet

    try:
        while rclpy.ok():
            key = get_key(settings)
            
            # 1. Hedef Kuvveti Belirle
            if key == 'd' or key == '\x1b[C':   # Sağa basılı tutuluyor
                target_force = MAX_FORCE
            elif key == 'a' or key == '\x1b[D': # Sola basılı tutuluyor
                target_force = -MAX_FORCE
            elif key == ' ':                    # BOŞLUK: Ani fren
                target_force = 0.0
                current_force = 0.0             # Fren için mevcut kuvveti anında sıfırla
            elif key == '\x03':                 # CTRL+C (Çıkış)
                break
            else:
                # Hiçbir tuşa basılmıyorsa hedefi 0 yap (Yavaşlama durumu)
                target_force = 0.0

            # 2. Mevcut Kuvveti Hedefe Doğru Kademeli Olarak Yaklaştır
            if current_force < target_force:
                current_force += FORCE_STEP
                if current_force > target_force: # Hedefi aşmasını engelle
                    current_force = target_force
            elif current_force > target_force:
                current_force -= FORCE_STEP
                if current_force < target_force: # Hedefi aşmasını engelle
                    current_force = target_force

            # 3. Ekrana düzgün bir şekilde yazdır
            print(f"\rHedef: {target_force:>6.1f} N | Uygulanan Kuvvet: {current_force:>6.1f} N   ", end="")

            # 4. Kuvveti yayınla
            wrench_msg = Wrench()
            wrench_msg.force.x = float(current_force)
            publisher.publish(wrench_msg)

    except Exception as e:
        print(f"\nHata oluştu: {e}")
    finally:
        # Çıkarken robotu güvenlice durdur
        stop_msg = Wrench()
        stop_msg.force.x = 0.0
        publisher.publish(stop_msg)
        
        termios.tcsetattr(sys.stdin, sys.stdin.fileno(), settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
