#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import select
import tty
import termios
from std_srvs.srv import Empty

# Stocker les paramètres originaux du terminal
original_terminal_settings = termios.tcgetattr(sys.stdin)

# Instructions affichées à l'utilisateur
msg = """
Control Your Drone!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : takeoff
g : land

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

# Mappage des touches aux mouvements (vx, vy, vz, vth_z)
# vx: avant/arrière, vy: gauche/droite (pour drone), vz: haut/bas, vth_z: rotation lacet
move_bindings = {
    'i': (1, 0, 0, 0),  # Avant
    'o': (1, 0, 0, -1), # Avant + rotation droite
    'j': (0, 0, 0, 1),  # Rotation gauche
    'l': (0, 0, 0, -1), # Rotation droite
    'u': (1, 0, 0, 1),  # Avant + rotation gauche
    ',': (-1, 0, 0, 0), # Arrière
    '.': (-1, 0, 0, 1), # Arrière + rotation gauche
    'm': (-1, 0, 0, -1),# Arrière + rotation droite
    # Pour mouvement latéral si votre drone le supporte (vy)
    # 'J': (0, 1, 0, 0),  # Gauche
    # 'L': (0, -1, 0, 0), # Droite
    # Pour mouvement vertical (vz)
    # 'I': (0, 0, 1, 0),  # Haut
    # 'M': (0, 0, -1, 0), # Bas
}

# Mappage des touches pour le contrôle de la vitesse
speed_bindings = {
    'q': (1.1, 1.1), # Augmenter vitesse linéaire et angulaire
    'z': (.9, .9),   # Diminuer vitesse linéaire et angulaire
    'w': (1.1, 1),   # Augmenter vitesse linéaire seulement
    'x': (.9, 1),    # Diminuer vitesse linéaire seulement
    'e': (1, 1.1),   # Augmenter vitesse angulaire seulement
    'c': (1, .9),    # Diminuer vitesse angulaire seulement
}

# Mappage des touches pour des actions spécifiques (décollage/atterrissage)
action_bindings = {
    't': 'takeoff',
    'g': 'land',
}

def get_key(settings):
    """Lit une touche du clavier de manière non bloquante."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Timeout de 0.1s
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_velocities(speed, turn):
    """Affiche les vitesses actuelles."""
    print(f"Currently:\tspeed {speed:.2f}\tturn {turn:.2f} ")

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Changez '/cmd_vel' si besoin

        # Clients pour les services de décollage et d'atterrissage
        # Remplacez 'drone_namespace' par le namespace réel de votre drone si applicable
        # et les noms de service par ceux de votre drone (ex: /takeoff, /land)
        self.takeoff_client = self.create_client(Empty, '/drone/takeoff') # Exemple
        self.land_client = self.create_client(Empty, '/drone/land')       # Exemple

        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting again...')
        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Land service not available, waiting again...')


        self.speed = 0.5  # Vitesse linéaire initiale/max
        self.turn = 1.0   # Vitesse angulaire initiale/max
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0

        self.get_logger().info(msg)
        print_velocities(self.speed, self.turn)

    def run_teleop(self):
        global original_terminal_settings # Pour pouvoir le restaurer
        try:
            while rclpy.ok():
                key = get_key(original_terminal_settings)
                if key:
                    if key in move_bindings:
                        self.x = move_bindings[key][0]
                        self.y = move_bindings[key][1] # Pour mouvement latéral (si activé)
                        self.z = move_bindings[key][2] # Pour mouvement vertical (si activé)
                        self.th = move_bindings[key][3]
                        self.status += 1
                    elif key in speed_bindings:
                        self.speed = self.speed * speed_bindings[key][0]
                        self.turn = self.turn * speed_bindings[key][1]
                        print_velocities(self.speed, self.turn)
                        self.status += 1
                    elif key in action_bindings:
                        action = action_bindings[key]
                        if action == 'takeoff':
                            self.get_logger().info("Requesting takeoff...")
                            self.takeoff_client.call_async(Empty.Request())
                        elif action == 'land':
                            self.get_logger().info("Requesting land...")
                            self.land_client.call_async(Empty.Request())
                        self.status += 1
                    elif key == ' ' or key == 'k':
                        self.x = 0.0
                        self.y = 0.0
                        self.z = 0.0
                        self.th = 0.0
                        self.get_logger().info("STOP")
                        self.status += 1
                    else:
                        self.x = 0.0
                        self.y = 0.0
                        self.z = 0.0
                        self.th = 0.0
                        if (key == '\x03'): # CTRL-C
                            break
                        self.status += 1 # Pour réafficher les vitesses

                    if self.status > 0: # Si une touche pertinente a été pressée
                        print_velocities(self.speed, self.turn)
                        self.status = 0 # Réinitialiser le compteur de statut

                else: # Si aucune touche n'est pressée, on peut réduire progressivement la vitesse
                    # Ou maintenir la dernière commande si c'est le comportement souhaité
                    # Pour cet exemple, on arrête si aucune touche n'est pressée pendant le timeout de get_key
                    self.x *= 0.9 # Décélération douce
                    self.y *= 0.9
                    self.z *= 0.9
                    self.th *= 0.9
                    # Si on veut un arrêt net quand on relâche :
                    # self.x = 0.0
                    # self.y = 0.0
                    # self.z = 0.0
                    # self.th = 0.0


                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed # Pour mouvement latéral
                twist.linear.z = self.z * self.speed # Pour mouvement vertical
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.publisher_.publish(twist)

                # Optionnel: permet à d'autres callbacks ROS de s'exécuter si nécessaire
                # rclpy.spin_once(self, timeout_sec=0.001)

        except Exception as e:
            self.get_logger().error(f"Exception in teleop loop: {e}")
            import traceback
            traceback.print_exc()

        finally:
            # Toujours restaurer les paramètres du terminal à la fin
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
            # Publier une commande d'arrêt à la fin
            twist = Twist()
            self.publisher_.publish(twist)
            self.get_logger().info("Keyboard teleop node stopped. Terminal settings restored.")


def main(args=None):
    global original_terminal_settings
    original_terminal_settings = termios.tcgetattr(sys.stdin) # Sauvegarder avant de faire quoi que ce soit

    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        node.run_teleop()
    except KeyboardInterrupt:
        pass # Géré dans run_teleop ou ici par la sortie de la boucle
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception in main: {e}")
    finally:
        # S'assurer que le terminal est restauré même en cas d'erreur non gérée avant la boucle principale
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
