#!/usr/bin/env python3

import rospy
import FreeSimpleGUI as sg
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



# --- Configurações de Estilo Global ---
THEME_BG = '#003f5c'  # Fundo geral escuro
THEME_TEXT = '#d0f0fd' # Texto claro azulado
ELEMENT_BG = '#002b3e' # Fundo dos elementos
BUTTON_OFF_COLOR = ('#d0f0fd', '#005678') # Cor do botão desligado (texto, fundo)
BUTTON_ON_COLOR = ('white', '#00a3d9')   # Cor do botão ligado (exemplo)
BORDER_COLOR = '#00a3d9' # Cor azul ciano para bordas e destaques
INNER_COLOR = "#000000"  # Cor interna para gráficos
KEY_ON_COLOR = '#00a3d9'
KEY_OFF_COLOR = BORDER_COLOR
KEY_TEXT_COLOR = 'white'


sg.theme_add_new('ROVTheme', {
    'BACKGROUND': THEME_BG,
    'TEXT': THEME_TEXT,
    'INPUT': ELEMENT_BG,
    'TEXT_INPUT': THEME_TEXT,
    'SCROLL': ELEMENT_BG,
    'BUTTON': BUTTON_OFF_COLOR,
    'PROGRESS': (BORDER_COLOR, ELEMENT_BG),
    'BORDER': 1,
    'SLIDER_DEPTH': 0,
    'PROGRESS_DEPTH': 0,
})
sg.theme('ROVTheme')



class InterfaceNode:
    def __init__(self):
        rospy.init_node("interface_node")

        # ================== PARÂMETROS ==================
        self.step_linear = 0.1
        self.step_angular = 0.1

        self.max_linear = 1.0
        self.max_angular = 1.0

        self.min_depth = 0.0
        self.max_depth = 10.0

        # ================== ROS ==================
        self.cmd_pub = rospy.Publisher(
            "cmd_vel",
            Twist,
            queue_size=10
        )

        rospy.Subscriber("/state", Odometry, self.state_callback)

        # ================== COMANDOS ==================
        self.cmd_velocity = 0.0
        self.cmd_rotation = 0.0
        self.cmd_depth = 0.0

        # ================== DADOS DOS SENSORES ==================
        self.depth = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        self.depth_marker_id = None  # ID do marcador na régua
        self.tilt_marker_id = None   # ID do marcador no gráfico de tilt
        self.horizon_line_id = None  # ID da linha do horizonte

        # ================== GRÁFICOS ==================

        # Régua de profundidade
        self.graph_depth = sg.Graph(
            canvas_size=(60, 300),
            graph_bottom_left=(0, 0),
            graph_top_right=(60, 100),
            background_color=ELEMENT_BG,
            key='-GRAPH-DEPTH-'
        )

        # Horizonte Artificial
        self.graph_horizon = sg.Graph(
            canvas_size=(300, 220),
            graph_bottom_left=(-150, -110),
            graph_top_right=(150, 110),
            background_color=THEME_BG,
            key='-GRAPH-HORIZON-'
        )

        # Tilt
        self.graph_tilt = sg.Graph(
            canvas_size=(150, 150),
            graph_bottom_left=(-50, -50),
            graph_top_right=(50, 50),
            background_color=ELEMENT_BG,
            key='-GRAPH-TILT-'
        )

        # HUD Teclas
        self.graph_keys = sg.Graph(
            canvas_size=(200, 150),
            graph_bottom_left=(0, 0),
            graph_top_right=(200, 150),
            background_color=ELEMENT_BG,
            key='-GRAPH-KEYS-'
        )

        # Comandos ROV
        self.cmd_panel = sg.Frame(
            'COMANDOS',
            [
                [sg.Text('VELOCIDADE', size=(14, 1)),
                sg.Text('0.00', key='-CMD-VEL-', font=('Arial', 20, 'bold'))],

                [sg.Text('ROTAÇÃO', size=(14, 1)),
                sg.Text('0.00', key='-CMD-ROT-', font=('Arial', 20, 'bold'))],

                [sg.Text('PROFUNDIDADE', size=(14, 1)),
                sg.Text('0.00', key='-CMD-DEP-', font=('Arial', 20, 'bold'))],
            ],
            background_color=ELEMENT_BG,
            pad=(10, 10)
        )

        # ================== LAYOUT ==================
        layout = [
            [
                sg.Column([
                    [self.graph_depth],
                    [sg.Text('0.0 m', key='-TXT-DEPTH-', justification='c')],
                    [sg.Text('ROV DEPTH', font=('Any', 8))]
                ]),

                sg.Column([
                    [self.graph_horizon],
                    [self.graph_tilt]
                ], element_justification='c'),

                sg.Column([
                    [self.cmd_panel],
                    [self.graph_keys]
                ], element_justification='c')
            ]
        ]

        self.window = sg.Window(
            'ROV - Interface Simplificada',
            layout,
            finalize=True,
            resizable=True,
            return_keyboard_events=True
        )


        self.key_state = {
            'w': False,
            'a': False,
            's': False,
            'd': False,
            'q': False,
            'e': False
        }

        self.key_drawings = {}

        self.draw_static_elements()
        

    # ================= ROS CALLBACKS =================

    def state_callback(self, msg):
        self.depth = msg.pose.pose.position.z
        _, self.roll, self.pitch = self.quat_to_yaw_roll_pitch(msg.pose.pose.orientation)

    # ================= UTIL =================

    def quat_to_yaw_roll_pitch(self, q):
        """
        Converte quaternion (w, x, y, z) para yaw, roll e pitch (em graus)

        Retorno:
            yaw   -> rotação em torno de Z
            roll  -> rotação em torno de X
            pitch -> rotação em torno de Y
        """

        # Roll (X-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)

        # Proteção contra erro numérico
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (Z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Converte para graus (opcional)
        yaw = math.degrees(yaw)
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)

        return yaw, roll, pitch

    def clamp(self, v, vmin, vmax):
        return max(min(v, vmax), vmin)


# ================== DESENHOS FIXOS ==================
    def draw_static_elements(self):

        # ----- Régua de profundidade -----
        g = self.graph_depth
        for y in range(0, 101, 10):
            g.draw_line((0, y), (15, y), BORDER_COLOR)
            g.draw_text(str(y/10), (35, y), color=BORDER_COLOR, font=('Any', 8))
        g.draw_line((50, 0), (50, 100), BORDER_COLOR, width=2)

        # ----- Horizonte -----
        h = self.graph_horizon
        h.draw_circle((0, 0), 80, BORDER_COLOR)
        h.draw_line((-75, 0), (75, 0), 'black', width=2)

        # ----- Tilt -----
        t = self.graph_tilt
        t.draw_line((-50, 0), (50, 0), BORDER_COLOR)
        t.draw_line((0, -50), (0, 50), BORDER_COLOR)

        # ----- Teclas -----
        self.draw_keys()

    def draw_keys(self):
        g = self.graph_keys
        g.erase()

        def key(x, y, label):
            g.draw_rectangle((x, y), (x + 40, y + 40),
                             line_color=BORDER_COLOR)
            g.draw_text(label, (x + 20, y + 20),
                        color='white',
                        font=('Arial', 12, 'bold'))

        key(80, 90, 'W')
        key(30, 40, 'A')
        key(80, 40, 'S')
        key(130, 40, 'D')

        key(30, 90, 'Q')
        key(130, 90, 'E')


# ================== DESENHOS MOVEIS ==================
    def update_measured_depth(self, depth_m):
        """
        Atualiza o valor numérico e o marcador da régua de profundidade.
        depth_m: profundidade em metros
        """

        # Limite da régua (exemplo: 0 a 10 m)
        max_depth = 10.0

        # Normaliza para escala do Graph (0–100)
        y = max(0, min(100, (depth_m / max_depth) * 100))

        g = self.graph_depth

        # Remove marcador antigo
        if self.depth_marker_id is not None:
            g.delete_figure(self.depth_marker_id)

        # Desenha marcador triangular
        self.depth_marker_id = g.draw_polygon(
            [(48, y), (60, y + 3), (60, y - 3)],
            fill_color='yellow'
        )

        # Atualiza texto
        self.window['-TXT-DEPTH-'].update(f'{depth_m:.2f} m')
    
    def update_tilt(self, pitch_deg, roll_deg):
        """
        Atualiza o gráfico de Tilt usando pitch e roll (em graus)
        """

        # Limites de visualização
        max_angle = 30.0  # graus
        max_xy = 45       # limite do gráfico

        # Normalização
        x = max(-max_xy, min(max_xy, (roll_deg / max_angle) * max_xy))
        y = max(-max_xy, min(max_xy, (pitch_deg / max_angle) * max_xy))

        g = self.graph_tilt

        # Remove marcador anterior
        if self.tilt_marker_id is not None:
            g.delete_figure(self.tilt_marker_id)

        # Desenha novo marcador
        self.tilt_marker_id = g.draw_circle(
            (x, y),
            5,
            fill_color='yellow'
        )   
    
    def update_horizon(self, pitch_deg, roll_deg):
        """
        Atualiza o horizonte artificial usando pitch e roll (graus)
        """

        g = self.graph_horizon

        # Apaga linha anterior
        if self.horizon_line_id is not None:
            g.delete_figure(self.horizon_line_id)

        # Limites visuais
        max_pitch = 30.0      # graus
        max_offset = 40       # pixels verticais
        line_half_length = 60

        # Pitch → deslocamento vertical
        offset_y = max(-max_offset, min(max_offset,
                        (pitch_deg / max_pitch) * max_offset))

        # Roll → rotação
        roll_rad = math.radians(roll_deg)

        # Linha base (antes da rotação)
        x1, y1 = -line_half_length, offset_y
        x2, y2 =  line_half_length, offset_y

        # Rotação em torno do centro
        cos_r = math.cos(roll_rad)
        sin_r = math.sin(roll_rad)

        x1r = x1 * cos_r - y1 * sin_r
        y1r = x1 * sin_r + y1 * cos_r

        x2r = x2 * cos_r - y2 * sin_r
        y2r = x2 * sin_r + y2 * cos_r

        # Desenha nova linha
        self.horizon_line_id = g.draw_line(
            (x1r, y1r),
            (x2r, y2r),
            color='yellow',
            width=3
        )

   
   
    # ================== INPUT TECLAS ==================
    def input_key_state(self):
    # Atualiza comandos (valores de exemplo)
        step_linear = self.step_linear
        step_angular = self.step_angular

        if self.key_state['w']:
            self.cmd_velocity += step_linear
        if self.key_state['s']:
            self.cmd_velocity -= step_linear

        if self.key_state['d']:
            self.cmd_rotation += step_angular
        if self.key_state['a']:
            self.cmd_rotation -= step_angular

        if self.key_state['e']:
            self.cmd_depth += step_linear
        if self.key_state['q']:
            self.cmd_depth -= step_linear
        # Limita valores
        self.cmd_velocity = self.clamp(self.cmd_velocity, -self.max_linear, self.max_linear)
        self.cmd_rotation = self.clamp(self.cmd_rotation, -self.max_angular, self.max_angular)
        self.cmd_depth = self.clamp(self.cmd_depth, self.min_depth, self.max_depth)


        # rospy.loginfo(f"Comandos: Vel {self.cmd_velocity:.2f} | Rot {self.cmd_rotation:.2f} | Dep {self.cmd_depth:.2f}")
        # Atualiza textos
        self.window['-CMD-VEL-'].update(f'{self.cmd_velocity:.2f}')
        self.window['-CMD-ROT-'].update(f'{self.cmd_rotation:.2f}')
        self.window['-CMD-DEP-'].update(f'{self.cmd_depth:.2f}')
        # Redesenha HUD
        self.draw_keys()

        cmd = Twist()
        cmd.linear.x = self.cmd_velocity
        cmd.linear.z = self.cmd_depth
        cmd.angular.z = self.cmd_rotation

        self.cmd_pub.publish(cmd)

    
    
    # ================= LOOP =================

    def run(self):

        while not rospy.is_shutdown():
            event, _ = self.window.read(timeout=50)

            if event == sg.WIN_CLOSED:
                break
            for key in self.key_state:
                self.key_state[key] = False

            # Captura teclas
            if isinstance(event, str):
                key = event.split(':')[0].lower()
                if key == 'space':
                    self.cmd_velocity = 0.0
                    self.cmd_rotation = 0.0
                    
                if key in self.key_state:
                    self.key_state[key] = True
                
                
            self.update_measured_depth(self.cmd_depth)

            pitch = self.pitch + self.cmd_rotation  # Exemplo de variação
            roll = self.roll + self.cmd_velocity # Exemplo de variação

            self.update_tilt(pitch, roll)
            self.update_horizon(pitch, roll)
            self.input_key_state()


        self.window.close()


if __name__ == "__main__":
    try:
        app_rov = InterfaceNode()
        app_rov.run()
    except rospy.ROSInterruptException:
        pass
