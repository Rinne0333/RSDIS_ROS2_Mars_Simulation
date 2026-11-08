# RSDIS | Histórico técnico — Perseverance e Mars Pathfinder

**Projeto:** RSDIS — Simulação de Marte com ROS 2 e Gazebo  
**Âmbito:** rover **NASA Perseverance**, modelo **Mars Pathfinder**, navegação e odometria associada  
**Workspace utilizado nos testes mais recentes:** `~/RSDIS_ROS2_Mars_Simulation_sync`  
**Ambiente:** ROS 2 Jazzy na Distrobox, `ROS_DOMAIN_ID=30`, Gazebo (mundo `mars`)  
**Atualizado em:** 08-11-2026  
**Estado:** histórico de desenvolvimento e diagnóstico, **não** relatório de funcionalidade concluída

> **Regra de leitura:** neste documento, **confirmado** significa observado em código, SDF ou saída de terminal; **proposto** significa que a alteração foi recomendada mas ainda não se verificou a sua execução; **por validar** significa que falta um teste conclusivo. É essencial manter estas distinções quando o projeto for retomado.

---

## 1. Objetivo do trabalho

Implementar a navegação autónoma do rover Perseverance até um ponto de interesse representado pelo modelo **Mars Pathfinder** no mundo do Gazebo.

O rover deve:

1. Receber as coordenadas de destino no mesmo referencial utilizado pela sua localização.
2. Saber a sua posição e orientação atuais.
3. Calcular a distância e o ângulo para o Pathfinder.
4. Rodar até ficar orientado para o destino e avançar até à tolerância definida.
5. Parar perante obstáculos e, numa fase posterior, contorná-los de forma autónoma.

A integração futura com um sistema externo que descubra ou envie as coordenadas do Pathfinder faz parte da missão global, mas **a implementação desse sistema externo está fora do âmbito deste registo**.

## 2. Organização relevante do projeto

Os caminhos seguintes são relativos à raiz do workspace `RSDIS_ROS2_Mars_Simulation_sync`.

| Componente | Caminho / identificação | Função |
|---|---|---|
| Mundo de Marte | `src/spaceros_gz_demos/spaceros_gz_demos/worlds/mars.sdf` | Terreno, rover e modelo Pathfinder. |
| Lançamento | `src/spaceros_gz_demos/spaceros_gz_demos/launch/mars.launch.xml` | Arranque da simulação e interfaces ROS/Gazebo. |
| Modelo Perseverance | `src/spaceros_gz_demos/spaceros_gz_demos/models/nasa_perseverance/model.sdf` | Articulações, rodas, sensores e plugins de mobilidade/odometria. |
| Modelo Pathfinder | `src/spaceros_gz_demos/spaceros_gz_demos/models/mars_pathfinder/` *(confirmar localização exata se reorganizado)* | Modelo separado com `model.config`, `model.sdf` e `meshes/model.dae`. |
| Pacote de navegação | `src/perseverance_navigation/` | Controlador de navegação do rover em C++. |
| Lógica de navegação | `src/perseverance_navigation/src/navigation.cpp` | Odometria, destino, obstáculos e publicação de velocidades. |
| Definição da classe | `src/perseverance_navigation/include/perseverance_navigation/navigation.hpp` | Parâmetros, estado e interfaces. |
| Entrada do executável | `src/perseverance_navigation/src/navigation_node.cpp` | Criação/execução do nó. |

> Os modelos do repositório de base provêm do projeto `david-dorf/spaceros_gz_demos`. Os caminhos acima refletem o workspace e os ficheiros mostrados ao longo dos testes; não são uma auditoria de todo o repositório atual.

## 3. Cronologia das alterações e descobertas

### 25-10-2026 | Primeiros testes de mobilidade

- **Confirmado:** o rover `nasa_perseverance` estava incluído no mundo `mars`.
- **Confirmado:** o plugin `gz::sim::systems::DiffDrive` era utilizado no `model.sdf`.
- **Confirmado:** inicialmente, o plugin acionava apenas as duas rodas dianteiras, através de `LF_DRIVE` e `RF_DRIVE`.
- **Confirmado:** um comando Gazebo Transport diretamente no tópico do modelo permitiu deslocar o rover:

  ```bash
  gz topic -t /model/nasa_perseverance/cmd_vel \
    -m gz.msgs.Twist -p 'linear: {x: 0.5}'
  ```

- **Diagnóstico inicial:** era necessário garantir a ligação do comando de velocidade ROS 2 ao Gazebo antes de desenvolver o controlo automático.
- **Configuração de referência do DiffDrive:** `wheel_separation=2.125`, `wheel_radius=0.526`.

### 05-11-2026 | Teleoperação e estrutura do pacote de navegação

- **Confirmado no histórico de desenvolvimento:** foi trabalhado um controlador de teleoperação C++ para publicar em `/perseverance/cmd_vel` (frequência de publicação de 20 Hz e paragem por timeout de teclas, no protótipo anterior).
- **Confirmado no histórico:** iniciou-se o pacote `perseverance_navigation` em C++, com classe própria, configuração por parâmetros ROS 2 e execução através de `ros2 run`.
- **Problemas de construção encontrados:** o pacote inicialmente não expunha o executável (`No executable found`) e faltava completar a configuração do alvo no `CMakeLists.txt`. Também ocorreram erros de compilação em versões preliminares do nó.
- **Resultado posterior:** o executável `navigation_node` passou a ser executável e produziu logs de controlo. Isto confirma que o pacote chegou a compilar em testes posteriores, mas **não confirma que a versão mais recente do código já tenha todas as correções desejadas**.

### 07-11-2026 | Inclusão do Mars Pathfinder

- **Confirmado:** o mundo utilizado é `mars.sdf`; o `mars.launch.xml` carrega esse mundo.
- **Primeira implementação:** foi utilizado um Pathfinder **simplificado**, definido diretamente no `mars.sdf` por um bloco `<model name="mars_pathfinder">`, com `static=true` e geometria simples (corpo e painéis).
- **Testes iniciais de colocação:** foram discutidas posições de teste próximas do rover, incluindo `(8, 4, 0)` e uma experiência com altura temporária para confirmar a visualização do objeto.
- **Último conteúdo do mundo diretamente confirmado nessa fase:** o modelo simplificado ainda aparecia no `mars.sdf` com:

  ```xml
  <model name="mars_pathfinder">
      <static>true</static>
      <pose>125 4 6.25 0 0 0</pose>
      <!-- Geometria simplificada -->
  </model>
  ```

- **Confirmado:** foi preparada uma alternativa mais detalhada e separada, na pasta `mars_pathfinder/`, com:

  ```text
  mars_pathfinder/
  ├── model.config
  ├── model.sdf
  └── meshes/
      └── model.dae
  ```

  A malha `model.dae` tinha cerca de **5,9 MB** no registo apresentado.
- **Proposto, não confirmado como estado final do mundo:** substituir o bloco simplificado por um `<include>` do modelo externo:

  ```xml
  <include>
      <uri>model://mars_pathfinder</uri>
      <pose>125 4 6.25 0 0 0</pose>
  </include>
  ```

  É necessário consultar o `mars.sdf` atual para verificar se esta substituição foi efetivamente aplicada. **Não assumir que o modelo detalhado já substituiu o simplificado.**

### 07–08-11-2026 | Navegação até um destino e deteção de obstáculos

- **Confirmado por uma versão de `navigation.cpp` partilhada:** o nó tem parâmetros ROS 2 para destino (`target_x`, `target_y`), ganhos proporcionais, velocidades máximas, tolerâncias e deteção de obstáculos.
- O cálculo de navegação considera o plano XY:

  ```cpp
  dx = target_x_ - x_;
  dy = target_y_ - y_;
  distance_error_ = std::hypot(dx, dy);
  target_yaw = std::atan2(dy, dx);
  angle_error_ = std::atan2(std::sin(target_yaw - yaw_),
                            std::cos(target_yaw - yaw_));
  ```

- O controlador gera velocidade angular proporcional ao erro de orientação, limitada por `max_angular_speed`; a velocidade linear só é ativada se `abs(angle_error) <= angle_tolerance`.
- **Confirmado:** subscrição da nuvem de pontos da câmara de profundidade em `/perseverance/depth_camera/points` (`sensor_msgs/msg/PointCloud2`).
- A deteção utiliza uma região frontal (distância, largura e altura configuráveis), um número mínimo de pontos e, na versão analisada, contadores separados para setores esquerdo, frontal e direito.
- **Comportamento implementado na versão observada:** parar perante um obstáculo. **Desvio autónomo/Bug2 ainda não foi demonstrado como implementado e validado.**
- A máquina de estados iniciou a estrutura `IDLE → GO_TO_GOAL`; `FOLLOW_OBSTACLE` foi discutido como próximo passo, não devendo ser tratado como comportamento concluído.
- **Problema de sensor observado nos logs:** avisos `Depth camera timeout` e necessidade de dados recentes para autorizar movimento.
- **Destino utilizado em testes de navegação:** `target_x=124.0`, `target_y=4.0`.

### 08-11-2026 | Diagnóstico da rotação e da odometria

**Sintoma inicial:** ao mandar o rover apontar para o Pathfinder, o erro angular reportado pelo nó diminuía de aproximadamente ±3 rad até perto de zero, mas o rover **não executava fisicamente a rotação correspondente**. A partir daí, o controlador começava a ordenar movimento linear numa direção incorreta.

**Teste de rotação manual, com o nó de navegação desligado:**

```bash
ros2 topic pub --rate 10 --times 60 \
  /perseverance/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: -0.5}}'

ros2 topic pub --once \
  /perseverance/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Uma das medições registou:

| Fonte | Antes | Depois | Variação aproximada |
|---|---:|---:|---:|
| Gazebo: componente yaw de RPY | `2.748310 rad` | `2.651860 rad` | `-0.096 rad` (`-5,5°`) |
| Odometria DiffDrive: yaw extraído do quaternion | `+0.0323 rad` | `-0.863 rad` | `-0.895 rad` (`-51°`) |

**Conclusão fundamentada:** nessa experiência, a rotação estimada pela odometria de rodas foi muito superior à variação angular física indicada pelo Gazebo. Isto é **compatível com** escorregamento, modelação inadequada da mobilidade ou diferenças de referenciais. Não permite atribuir com certeza o problema a uma única causa.

#### Alteração do DiffDrive para seis rodas

- **Confirmado no SDF partilhado:** o plugin foi alterado para acionar as seis articulações existentes:

  ```xml
  <left_joint>LF_DRIVE</left_joint>
  <left_joint>LM_DRIVE</left_joint>
  <left_joint>LR_DRIVE</left_joint>

  <right_joint>RF_DRIVE</right_joint>
  <right_joint>RM_DRIVE</right_joint>
  <right_joint>RR_DRIVE</right_joint>
  ```

- Foram mantidos `wheel_separation=2.125`, `wheel_radius=0.526` e os restantes limites de velocidade/aceleração então existentes.
- As quatro articulações de direção dianteiras/traseiras `LF_STEER`, `RF_STEER`, `LR_STEER`, `RR_STEER` estavam definidas como `fixed`. Logo, a mudança de direção funciona por diferença de velocidade entre os lados (tipo *skid steering*); este compromisso pode introduzir deslizamento lateral.
- **Estado:** alteração aplicada ao SDF. **A correção da rotação física não foi confirmada.**

#### Discrepância entre odometria estimada e posição no mundo

Numa medição posterior, obtiveram-se os seguintes valores:

| Fonte | X (m) | Y (m) | Z (m) |
|---|---:|---:|---:|
| Gazebo: `gz model -m nasa_perseverance --pose` | `-4.387030` | `-1.449670` | `2.744620` |
| ROS 2: `/perseverance/odometry` | `-5.839686` | `0.449624` | `0.0` |

**Interpretação:** as leituras não estavam diretamente alinhadas. Pode existir erro por integração/escorregamento, diferença de origem ou combinação dos dois. Comparar apenas valores absolutos não permite distinguir todos estes efeitos.

#### Introdução da odometria *ground truth*

Foi acrescentado um plugin independente ao `nasa_perseverance/model.sdf` para publicar a pose derivada do estado simulado:

```xml
<plugin
    filename="gz-sim-odometry-publisher-system"
    name="gz::sim::systems::OdometryPublisher">

    <odom_frame>world</odom_frame>
    <robot_base_frame>nasa_perseverance/base_link</robot_base_frame>
    <odom_topic>/perseverance/ground_truth/odometry</odom_topic>
    <odom_publish_frequency>20</odom_publish_frequency>
    <dimensions>3</dimensions>
</plugin>
```

**Incidentes de configuração resolvidos:**

1. Inicialmente, o `OdometryPublisher` foi colocado **dentro** do bloco XML do `DiffDrive`. Resultado: `No publishers` no tópico de *ground truth*.
2. Na tentativa de separar os plugins, ficou uma segunda cópia da configuração das rodas fora do seu bloco, incluindo um `</plugin>` indevido. Resultado: erro `xml.etree.ElementTree.ParseError: mismatched tag`.
3. O bloco duplicado foi identificado nas linhas **1644–1664** do ficheiro enviado e removido.
4. Após correção e reinício da simulação, o comando `gz topic -i -t /perseverance/ground_truth/odometry` passou a indicar **um publisher `gz.msgs.Odometry`**. **Confirmado.**

**Bridge usado para enviar a mensagem ao ROS 2:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/perseverance/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

**Validação numérica conclusiva da posição:**

| Eixo | Gazebo (m) | ROS 2 ground truth (m) | Diferença absoluta |
|---|---:|---:|---:|
| X | `0.827665` | `0.8275921883841085` | ~`0.073 mm` |
| Y | `-0.057421` | `-0.05738194254917492` | ~`0.039 mm` |
| Z | `3.179200` | `3.1791958727312113` | ~`0.004 mm` |

O Gazebo indicou, nesse instante aproximado, `RPY = (3.106360, 0.258675, 2.744010)` rad. As diferenças de posição são compatíveis com comandos executados em instantes distintos. **A publicação de localização global no ROS 2 ficou confirmada.**

#### Causa encontrada para a ausência de melhoria no controlador

Apesar da nova odometria funcionar, o rover continuava a apresentar o problema de orientação. A inspeção direta do código revelou:

```text
src/perseverance_navigation/src/navigation.cpp:222:
    "/perseverance/odometry",
```

Ou seja, **o controlador ainda estava subscrito à odometria antiga**. O ROS 2 confirmou também, num dos testes, `Subscription count: 0` em `/perseverance/ground_truth/odometry`.

**Correção recomendada no final da sessão (ainda sem confirmação de teste pós-alteração):**

```diff
-        "/perseverance/odometry",
+        "/perseverance/ground_truth/odometry",
```

Depois, recompilar o pacote `perseverance_navigation`, reiniciar o nó e verificar uma subscrição ativa ao novo tópico. **Não registar a navegação como resolvida antes desse teste.**

---

## 4. Configuração e interfaces da navegação

### 4.1. Fluxo de informação

```text
Gazebo: estado físico do nasa_perseverance
    |
    +--> DiffDrive: comanda 6 articulações, estima odometria das rodas
    |       +--> /perseverance/odometry                 [fonte antiga]
    |
    +--> OdometryPublisher: estado simulado do modelo
            +--> /perseverance/ground_truth/odometry  [Gazebo]
                         |
                         v
                    ros_gz_bridge
                         |
                         v
                  ROS 2: nav_msgs/Odometry
                         |
                         v
                  perseverance_navigation
                         |
                         v
                 /perseverance/cmd_vel
                         |
                         v
                       DiffDrive
```

**Nota:** este diagrama representa o fluxo pretendido após a alteração ao subscriber de `navigation.cpp`; na última inspeção confirmada, o código ainda subscrevia o tópico antigo.

### 4.2. Tópicos e papéis

| Tópico | Mensagem | Papel |
|---|---|---|
| `/perseverance/cmd_vel` | `geometry_msgs/msg/Twist` | Comando de velocidade linear/angular do rover. |
| `/perseverance/odometry` | `nav_msgs/msg/Odometry` | Estimativa de odometria associada ao DiffDrive. |
| `/perseverance/ground_truth/odometry` | `nav_msgs/msg/Odometry` após bridge | Pose baseada no estado simulado do modelo. |
| `/perseverance/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | Dados de profundidade para deteção frontal de obstáculos. |

### 4.3. Lógica observada em `navigation.cpp`

- `loadParameters()`: declara destino, ganhos, limites e configurações de deteção.
- `setupParameterCallbacks()`: valida e atualiza determinados parâmetros durante a execução.
- `setupRosInterfaces()`: configura subscribers/publisher e *timer* de controlo de **100 ms (10 Hz)**.
- `odometryCallback()`: lê `x`, `y` e converte quaternion para yaw.
- `odometryIsValid()`: exige odometria recebida recentemente (limite observado de **0,5 s**).
- `pointcloudCallback()`: conta pontos válidos em regiões de deteção.
- `calculateNavigationErrors()`: calcula distância euclidiana e erro angular normalizado para `[-π, π]`.
- `calculateVelocityCommand()`: controlador proporcional com saturações; só autoriza velocidade linear quando está suficientemente alinhado.
- `controlLoop()`: impede movimento com dados inválidos/antigos, gere `IDLE → GO_TO_GOAL`, pára perante obstáculos, publica comandos e regista diagnósticos.

**Limitação atual:** o controlador só faz navegação direta até ao ponto; evitar obstáculos, replanear trajetória e acompanhar atualizações do alvo precisam de implementação/testes adicionais.

### 4.4. Coordenadas do Pathfinder: atenção à divergência

Há dois valores documentados em momentos diferentes:

- **Pose efetivamente observada num `mars.sdf` em 07-11:** `125 4 6.25 0 0 0`.
- **Destino usado na CLI dos testes posteriores:** `(124.0, 4.0)`.

Também foi discutida uma variante de colocação perto de `(124, 4)` noutra edição do mundo, **sem confirmação de que esta passou a ser a versão final carregada**.

**Ação obrigatória antes do ensaio final:** ler a definição ativa de `mars_pathfinder` em `mars.sdf` ou medir a sua pose em Gazebo; alinhar `target_x` e `target_y` com o alvo real. Não assumir que `(124, 4)` coincide com o centro do modelo.

---

## 5. Estado atual em 08-11-2026

| Item | Estado | Evidência / observação |
|---|---|---|
| Mundo `mars` com Perseverance | **Confirmado** | Gazebo publica a pose de `nasa_perseverance`. |
| Pathfinder presente no mundo durante os testes | **Confirmado historicamente** | Bloco inline `mars_pathfinder` em `mars.sdf`; tipo atual (inline vs modelo detalhado) por verificar. |
| Pacote `perseverance_navigation` criado | **Confirmado** | Executável e logs de navegação observados em testes. |
| Comandos ROS 2 `/perseverance/cmd_vel` | **Confirmado** | Rover reage, embora a rotação física continue problemática. |
| DiffDrive configurado com seis rodas | **Confirmado no SDF enviado** | Seis nomes de juntas presentes. |
| Deteção frontal por profundidade | **Implementada no código analisado** | Alertas/timeout observados; contorno de obstáculos não confirmado. |
| Plugin `OdometryPublisher` separado e funcional | **Confirmado** | Gazebo apresentou um publisher no tópico novo. |
| Bridge de ground truth para ROS 2 | **Confirmado** | Valores X/Y/Z praticamente iguais aos do Gazebo. |
| `navigation.cpp` subscreve ground truth | **Pendente de aplicar/confirmar** | Último `grep` apontava para `/perseverance/odometry`, linha 222. |
| Erro angular acompanha o yaw físico | **Por validar** | Depende da correção de subscrição e de novo ensaio. |
| Rotação física suficiente para apontar ao Pathfinder | **Por validar** | Seis rodas, por si só, não resolveram conclusivamente. |
| Chegada autónoma ao Pathfinder | **Por validar** | Não foi demonstrada chegada à coordenada-alvo. |
| Contorno de obstáculos (Bug2 ou similar) | **Não concluído** | Apenas discutido como etapa posterior. |

---

## 6. Retoma do trabalho: ordem recomendada

### Passo A — Garantir a subscrição correta

Verificar a linha da odometria no código:

```bash
cd ~/RSDIS_ROS2_Mars_Simulation_sync

grep -n -E 'ground_truth/odometry|/perseverance/odometry' \
  src/perseverance_navigation/src/navigation.cpp
```

A configuração pretendida é:

```cpp
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/perseverance/ground_truth/odometry",
    rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odometryCallback(msg);
    }
);
```

**Alterar apenas o nome do tópico** se a versão atual ainda utilizar a odometria antiga.

### Passo B — Compilar o pacote

```bash
colcon build --packages-select perseverance_navigation --symlink-install
source install/setup.bash
```

Confirmar que terminou sem erros e que o processo anterior de navegação foi encerrado.

### Passo C — Executar Gazebo e bridge

Com o ambiente Jazzy ativo, iniciar normalmente o mundo `mars` a partir do launch do projeto. Garantir **uma única instância** do bridge para o tópico de ground truth:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/perseverance/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

O bridge tem de continuar em execução. Outros bridges que o `mars.launch.xml` já inicie podem ser necessários para os comandos e sensores, e não devem ser desligados indiscriminadamente.

### Passo D — Confirmar dados antes de comandar o rover

```bash
gz topic -i -t /perseverance/ground_truth/odometry

ros2 topic echo --once \
  /perseverance/ground_truth/odometry \
  --field pose.pose.position

gz model -m nasa_perseverance --pose
```

Esperado: publisher Gazebo presente e posição ROS 2 próxima da pose do modelo.

### Passo E — Executar e inspecionar navegação

```bash
ros2 run perseverance_navigation navigation_node \
  --ros-args -p target_x:=124.0 -p target_y:=4.0
```

> **O comando acima reproduz o destino usado nos testes, não garante a posição atual do Pathfinder.** Atualizar os dois parâmetros se a pose do objeto for diferente.

Noutro terminal, **enquanto o nó estiver a correr**:

```bash
ros2 node list
ros2 topic info /perseverance/ground_truth/odometry --verbose
```

Esperado: um publisher do `ros_gz_bridge` e **uma subscrição do nó de navegação**. Em testes anteriores, `ros2 node list` mostrou apenas dois nós `/ros_gz_bridge` e nenhum de navegação, o que não permitia diagnosticar o seu subscriber; verificar sempre com o nó ativo.

### Passo F — Ensaiar rotação antes de longas deslocações

Comparar **no mesmo ensaio**:

1. Yaw real indicado pelo Gazebo antes do comando.
2. Yaw real indicado pelo Gazebo quando o nó anuncia `Angle` próximo de zero.
3. Erro angular e velocidades `V/W` produzidos pelo controlador.
4. Pose X/Y do rover e coordenadas do destino no mesmo instante.

Se o erro angular chegar a zero mas a orientação real não apontar para o Pathfinder, verificar cálculo/quaternion/referenciais e orientação do eixo longitudinal do modelo. Se o erro permanecer elevado e o rover quase não rodar, investigar física de rodas, contactos e tração. Não compensar uma falha mecânica alterando arbitrariamente o ganho angular.

### Passo G — Funcionalidade posterior

Depois de validar localização, orientação e chegada a um objetivo estático: adicionar atualização dinâmica da coordenada do Pathfinder e comportamento de contorno de obstáculos, com estados e critérios de transição testáveis.

---

## 7. Problemas encontrados e respetiva resolução

| Sintoma / erro | Causa identificada | Resolução / situação |
|---|---|---|
| `No executable found` ao usar `ros2 run` | Configuração de construção incompleta numa fase inicial | CMake/target trabalhados; executável correu posteriormente. |
| Erro angular desce de ~3 rad para ~0 sem rotação visual equivalente | Divergência entre orientação estimada pelas rodas e movimento físico; depois verificou-se que `navigation.cpp` ainda subscrevia a odometria antiga | Corrigir subscriber e repetir ensaio; **resolução final pendente**. |
| `No publishers on topic /perseverance/ground_truth/odometry` | Plugin `OdometryPublisher` inicialmente aninhado dentro de `DiffDrive` | Separar os plugins. |
| `ParseError: mismatched tag` no `model.sdf` | Bloco duplicado de rodas e `</plugin>` extra nas linhas 1644–1664 da cópia enviada | Remoção do bloco duplicado; publicação apareceu após correção. |
| `Unknown topic` em ROS 2 para ground truth | Bridge não estava visível/ativo naquele instante | Iniciar bridge, manter execução e consultar o grafo ROS 2. |
| Dois nós `/ros_gz_bridge` com o mesmo nome | Mais de uma instância do bridge ativa | Evitar instâncias duplicadas desnecessárias; preservar bridges exigidos por outras interfaces. |
| `Subscription count: 0` em ground truth | Nenhum nó de navegação subscrevia o tópico naquele instante; código fonte usava odometria antiga | Alterar o subscriber para ground truth e confirmar durante execução. |
| `Depth camera timeout` | Ausência ou atraso de dados válidos da câmara de profundidade | Controlo pára por segurança; origem da ausência de dados ainda por investigar se persistir. |

## 8. Comandos úteis de diagnóstico

```bash
# Consultar posição e orientação físicas no mundo
gz model -m nasa_perseverance --pose

# Verificar quem publica a odometria ground truth no Gazebo
gz topic -i -t /perseverance/ground_truth/odometry

# Ler a localização recebida pelo ROS 2
ros2 topic echo --once \
  /perseverance/ground_truth/odometry \
  --field pose.pose.position

# Verificar os nós ativos
ros2 node list

# Verificar quem publica e quem subscreve a odometria
ros2 topic info /perseverance/ground_truth/odometry --verbose

# Descobrir qual a odometria referida no código de navegação
grep -RInE 'ground_truth/odometry|/perseverance/odometry' \
  src/perseverance_navigation/

# Validar a sintaxe XML do modelo Perseverance
python3 -c "import xml.etree.ElementTree as ET; ET.parse('src/spaceros_gz_demos/spaceros_gz_demos/models/nasa_perseverance/model.sdf'); print('XML válido')"
```

## 9. Referências técnicas para a implementação e o relatório

- **Gazebo Sim — DiffDrive:** documentação do plugin de acionamento diferencial, rodas e odometria.  
  <https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1DiffDrive.html>
- **Gazebo Sim — OdometryPublisher:** publicação da pose e odometria do modelo.  
  <https://gazebosim.org/api/sim/9/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html>
- **ROS 2 Jazzy — ros_gz_bridge:** ponte entre mensagens do Gazebo Transport e ROS 2.  
  <https://docs.ros.org/en/jazzy/p/ros_gz_bridge/>
- **ROS 2 — interfaces de mensagens `nav_msgs/Odometry`:** pose e velocidade do robô.  
  <https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html>
- **Repositório de origem da simulação:**  
  <https://github.com/david-dorf/spaceros_gz_demos>

---

## 10. Registo para a próxima sessão

**Última evidência concreta:** o `grep` mostrou `src/perseverance_navigation/src/navigation.cpp:222` com `"/perseverance/odometry"`. Foi instruída a alteração para `"/perseverance/ground_truth/odometry"`, mas **a execução e a validação pós-alteração não constam do histórico confirmado**.

**Primeira tarefa ao retomar:** verificar a linha do subscriber, compilar se necessário, iniciar apenas os processos ROS/Gazebo necessários e confirmar `Subscription count: 1` para o tópico de *ground truth* com o `navigation_node` ativo.

**Segunda tarefa:** testar a rotação até ao Pathfinder, comparando o yaw físico no Gazebo com o erro angular do controlador.

**Terceira tarefa:** confirmar a pose atual do Pathfinder no `mars.sdf` e corrigir a discrepância `125,4` versus `124,4` antes de declarar sucesso na navegação.

**Critério mínimo de conclusão:** o rover recebe pose global válida, orienta-se efetivamente para o alvo, aproxima-se dele e termina dentro da tolerância de distância configurada, sem comandar movimento caso a odometria ou os sensores necessários fiquem indisponíveis.
