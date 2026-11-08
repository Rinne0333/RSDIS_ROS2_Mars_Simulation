# ingenuity_navigation V3 | Missão autónoma A → B

Package C++17/ROS 2 Jazzy para simular o **Ingenuity** no Gazebo Harmonic.

## 1. O que faz

O `navigation_node` lê `/ingenuity/odometry` e coordena a missão:

```text
WAITING
   ↓   Odometria estável no solo, memoriza A e calcula B
TAKEOFF
   ↓   Rampa de subida, objetivo horizontal = A
HOVER_A
   ↓   Verifica XY/Z e velocidades durante tempo configurado
ROTATE
   ↓   Mantém A e altura; tenta rodar yaw para apontar a B
NAVIGATE
   ↓   Referência avança gradualmente na reta A → B
BRAKE
   ↓   PD reduz velocidade e aproxima-se precisamente de B
HOVER_B
   ↓   Verifica estabilidade horizontal e vertical
LANDING
   ↓   Rampa de descida no local B
COMPLETE
```

Há transições para `FAULT` quando a odometria falha, a altitude ou a
posição saem dos limites, ou um estado excede o tempo limite.

**Não considera atingido o destino só por atingir uma altitude.**

## 2. O que há em cada ficheiro

| Ficheiro | Papel |
|---|---|
| `include/ingenuity_navigation/flight_math.hpp` | Vetores, quaternions, conversões e heading/yaw do mundo. |
| `src/altitude_controller.cpp` | PID de Z → empuxo total (N). |
| `src/position_controller.cpp` | PD XY → referência de roll/pitch. |
| `src/attitude_controller.cpp` | PD roll/pitch → referências das duas swashplates. |
| `src/yaw_controller.cpp` | PD de yaw → diferença de empuxo (N). |
| `src/motor_mixer.cpp` | Divide empuxo total + diferencial entre os dois rotores. |
| `src/mission_manager.cpp` | Sequência de estados e metas da missão. |
| `src/navigation.cpp` | Integração ROS 2, parâmetros, odometria, watchdog e publishers. |
| `src/navigation_node.cpp` | Função `main`. |
| `config/mission.yaml` | Parâmetros ajustáveis sem recompilar. |
| `launch/mission.launch.py` | Forma alternativa de lançar a missão. |
| `test/controller_smoke_test.cpp` | Testes matemáticos sem Gazebo. |

## 3. Importante: yaw **não garantido** com o modelo original

O `Thruster` gera força segundo o eixo da junta. Um par de rotores que
rodam em sentidos opostos **não garante** que a diferença de empuxo crie
um binário aerodinâmico de yaw fisicamente controlável neste SDF.

Nesta V3, o `YawController` calcula um diferencial entre os comandos das
hélices. Os **sinais e o efeito físico** precisam de ser confirmados.

- `rotate_before_navigation: true`: exige rodar para B antes de avançar.
  Se o modelo não gerar binário, após `rotate_timeout` entra em FAULT.
- `rotate_before_navigation: false`: omite ROTATE e **mantém a orientação
  inicial**, enquanto o controlo XY pode deslocar-se lateralmente.

**Não há rotação virtual da pose nem teletransporte.** Se quiseres rotação
real e o diferencial de rotores não a produzir, será necessário adaptar
os atuadores / a física do modelo no Gazebo.

## 4. Limitação atual identificada nos logs

A versão V2 perdeu 1,65 m em XY durante a descolagem. A sequência de
estados não resolve, por si só, o problema físico dos swashplates.

Antes de confiar no voo: medir a **posição das juntas**, confirmar o
sentido de atuação de SW1/SW2, e afinar ganhos. Este projeto é apenas de
**simulação**; o estado FAULT envia comandos neutros e não garante
aterragem controlada.

## 5. Instalação na Distrobox Jazzy

Guarda o ZIP em `/home/alexandre-coelho/Downloads` no **host Ubuntu**.
Dentro da Distrobox:

```bash
cd ~/RSDIS_ROS2_Mars_Simulation_sync
cp -a src/ingenuity_navigation "$HOME/ingenuity_navigation_backup_$(date +%Y%m%d_%H%M%S)"
unzip -o /home/alexandre-coelho/Downloads/ingenuity_navigation_v3.zip -d src/
colcon build --packages-select ingenuity_navigation --symlink-install
source install/setup.bash
colcon test --packages-select ingenuity_navigation
colcon test-result --verbose
```

O ZIP contém `ingenuity_navigation/` como pasta raiz e deve ser extraído
em `src/`, não dentro de `src/ingenuity_navigation/`.

## 6. Experiência A → B

**Terminal 1: Gazebo**. Reiniciar o Gazebo após alterações de SDF, com
modelo físico de massa total ~2 kg e simulação despausada:

```bash
cd ~/RSDIS_ROS2_Mars_Simulation_sync
source install/setup.bash
ros2 launch spaceros_gz_demos mars.launch.xml
```

**Terminal 2: missão**. Começar pelo nó **desarmado**:

```bash
cd ~/RSDIS_ROS2_Mars_Simulation_sync
source install/setup.bash
ros2 run ingenuity_navigation navigation_node --ros-args \
  --params-file src/ingenuity_navigation/config/mission.yaml
```

Parar com `Ctrl+C`; reiniciar o Gazebo e executar missão armada.
Primeira experiência curta em +X:

```bash
ros2 run ingenuity_navigation navigation_node --ros-args \
  --params-file src/ingenuity_navigation/config/mission.yaml \
  -p arm:=true -p goal_dx:=2.0 -p goal_dy:=0.0 \
  -p flight_height:=1.5
```

Para exercitar uma **rotação ~90°** antes de avançar em +Y:

```bash
ros2 run ingenuity_navigation navigation_node --ros-args \
  --params-file src/ingenuity_navigation/config/mission.yaml \
  -p arm:=true -p goal_dx:=0.0 -p goal_dy:=2.0 \
  -p flight_height:=1.5 -p rotate_before_navigation:=true
```

A missão só inicia depois de capturar A com odometria estável. Não correr
`ingenuity_mission` ou outro nó que envie comandos aos mesmos atuadores.

**Terminal 3: diagnóstico**:

```bash
ros2 topic echo /ingenuity/navigation/state
ros2 topic echo /ingenuity/navigation/target
ros2 topic echo /ingenuity/odometry --field pose.pose.position
ros2 topic echo --once /ingenuity/joint_states
```

## 7. Parâmetros relevantes

| Parâmetro | Função |
|---|---|
| `arm` | `false` por omissão, impede empuxo. |
| `goal_dx`, `goal_dy` | Deslocamento relativo a A em **odom** (metros). |
| `flight_height` | Altura adicional acima de A (m). |
| `rotate_before_navigation` | Obriga a alinhar yaw antes da navegação. |
| `navigation_speed` | Velocidade da referência ao longo da reta (m/s). |
| `max_target_lead_m` | Impede a referência XY de se afastar demasiado. |
| `brake_radius` | Ativa BRAKE ao aproximar de B. |
| `kp_xy`, `kd_xy` | Ganhos de posição horizontal. |
| `kp_att`, `kd_att` | Ganhos de atitude roll/pitch. |
| `kp_yaw`, `kd_yaw`, `max_yaw_diff_n` | Ganhos e limite do diferencial de yaw. |
| `yaw_differential_sign` | Corrige sinal do diferencial se necessário. |
| `swashplate_1_sign`, `swashplate_2_sign` | Sentidos físicos dos atuadores. |
| `takeoff_xy_gain_scale` | Suavização de XY durante descolagem. |
| `max_takeoff_drift`, `max_xy_radius` | Limites de excursão. |

## 8. Referenciais e limites de segurança

- Posição e ponto B em `nasa_ingenuity/odom`.
- Odometria `twist` em `nasa_ingenuity/base_footprint`, rodado para o
  referencial do mundo antes de utilizar as velocidades XYZ.
- Yaw calculado a partir da direção de +X do corpo expressa no mundo.
- Yaw rate em Z **do mundo**; no modelo invertido em X o Z local tem sinal
  oposto.
- O eixo de empuxo é `-Z` do corpo no modelo original.
- PID integra apenas amostras novas com timestamp válido do Gazebo.
- Watchdog monotónico deteta falta de odometria mesmo quando o mundo pausa.
- `FAULT` neutraliza rotores e swashplates, mas não garante segurança
  física nem aterragem controlada. Executar exclusivamente na simulação.

## 9. Fontes bibliográficas

- [ROS 2 Jazzy: nav_msgs/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html)
- [ROS 2: Clock and Time](https://design.ros2.org/articles/clock_and_time.html)
- [Gazebo Sim 8: Thruster](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Thruster.html)
- [Gazebo Sim 8: JointPositionController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)
- [Feedback Systems, Åström & Murray](https://fbsbook.org/)
