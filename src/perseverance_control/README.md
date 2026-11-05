# Perseverance — controlo pelo teclado em C++ / ROS 2 Jazzy

Este pacote permite conduzir o rover do cenário Marte de spaceros_gz_demos.
Parte do nó simples do projeto: mantém um main, um publisher e um ciclo.
O teclado muda a mensagem enviada; não controla diretamente os motores.

## 1. O que acontece quando carregas numa tecla?

1. O terminal entrega um carácter, sem esperar por Enter.
2. update_command() transforma esse carácter numa velocidade.
3. O publisher envia uma mensagem Twist em /perseverance/cmd_vel.
4. A ponte ros_gz_bridge traduz a mensagem para o Gazebo.
5. O controlador do modelo recebe a velocidade em /model/nasa_perseverance/cmd_vel.

O nó não calcula trajetórias, não evita obstáculos e não controla o braço.
Também não controla o Ingenuity: esse modelo tem interfaces de voo diferentes.

## 2. Comandos

| Tecla | Movimento | linear.x (m/s) | angular.z (rad/s) |
|---|---|---:|---:|
| W | Avançar | +0,5 | 0 |
| S | Recuar | -0,5 | 0 |
| A | Rodar à esquerda | 0 | +0,5 |
| D | Rodar à direita | 0 | -0,5 |
| Espaço | Parar | 0 | 0 |
| Q / Ctrl+C | Enviar paragem e sair | 0 | 0 |

Aceita letras maiúsculas e minúsculas. Outras teclas normais param o rover.
As setas e as teclas de função não são comandos de movimento: as sequências
de escape são descartadas. Escape isolado para e pode consumir a tecla seguinte;
espera 0,8 s antes de retomar. Usa Espaço como comando habitual de paragem.
Ctrl+D também sai. Ctrl+Z fica desativado enquanto o nó está ativo.

**Mantém o terminal selecionado. Não carregues nas teclas na janela do Gazebo.**
Mantém uma tecla de movimento premida, com repetição automática do teclado
ativa, ou volta a premi-la. Sem novos caracteres durante 0,8 s, o nó envia zero.
Um terminal não informa quando largas uma tecla: a paragem pode demorar esse
intervalo, mais aproximadamente um ciclo. Não há movimento diagonal nem
combinação simultânea W+A; prevalece a última tecla interpretada.

Se o atraso de repetição do teclado for superior a 0,8 s, poderá haver uma
paragem breve antes de começar a repetição. Esta é uma implementação educativa
para simulação, não um dispositivo de comando com deteção de tecla libertada.

## 3. Ficheiros

| Ficheiro | Responsabilidade |
|---|---|
| src/perseverance_node.cpp | ROS 2, interpretação de teclas, velocidades, ciclo e saída |
| include/perseverance_control/keyboard.hpp | Leitura e restauro do terminal Linux |
| CMakeLists.txt | Compilar e instalar o executável |
| package.xml | Identificação e dependências do pacote |
| README.md | Este guia |

## 4. Instalar no projeto existente

O ZIP contém uma pasta perseverance_control. Extrai primeiro para uma pasta
temporária. Copia o conteúdo para o teu pacote existente, depois de fazeres
backup. Não cries src/perseverance_control/perseverance_control.

Dentro da Distrobox Jazzy, a pasta destino é:

    ~/RSDIS_ROS2_Mars_Simulation/src/perseverance_control/

Na configuração usada neste projeto, o caminho completo é:

    /home/alexandre-coelho/distrobox-homes/jazzy-home/RSDIS_ROS2_Mars_Simulation/src/perseverance_control/

Confirma com echo "$HOME": o ~ do Ubuntu e o ~ da Distrobox podem ser diferentes.
O nome certo do ficheiro é perseverance_node.cpp, não preseverance_node.cpp.
Se copiares apenas o .cpp, faltará keyboard.hpp: copia os quatro ficheiros de
código/configuração para as respetivas pastas.

Para fazer backup do pacote atual, antes de copiar:

```bash
cd ~/RSDIS_ROS2_Mars_Simulation
mkdir -p ~/rsdis_backups
cp -a src/perseverance_control ~/rsdis_backups/perseverance_control_$(date +%Y%m%d_%H%M%S)
```

O backup fica fora de src para não criar dois pacotes com o mesmo nome.
O maintainer do package.xml é um placeholder; troca-o pelos dados do grupo
antes de distribuir o projeto.

## 5. Compilar

Executa dentro da Distrobox Jazzy. Pressupõe ROS 2 Jazzy, colcon e os pacotes
rclcpp/geometry_msgs instalados, como no ambiente onde o nó anterior compilou.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
cd ~/RSDIS_ROS2_Mars_Simulation
colcon build --symlink-install --packages-select perseverance_control
```

Só depois de terminar com sucesso:

```bash
source install/local_setup.bash
ros2 pkg prefix perseverance_control
```

O prefixo deve corresponder ao install deste workspace.
Guardar o .cpp não atualiza o executável: recompila sempre após alterações,
mesmo usando --symlink-install.

### Se reaparecerem avisos de Lyrical

Carregar Jazzy não remove necessariamente caminhos herdados de Lyrical.
Para compilar e executar este teleop num ambiente limpo, abre esta shell
temporária **a partir de dentro da Distrobox**:

```bash
env -i HOME="$HOME" USER="$USER" TERM="$TERM" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 ROS_DOMAIN_ID=30 bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
cd ~/RSDIS_ROS2_Mars_Simulation
colcon build --symlink-install --packages-select perseverance_control --cmake-clean-cache
source install/local_setup.bash
```

Não abras a GUI do Gazebo nesta shell: foram removidas variáveis gráficas.
Usa-a para o teleop. exit regressa ao ambiente anterior; não corrige o .bashrc.
O local_setup carrega este workspace sobre o Jazzy já preparado, evitando a
cadeia de ambientes anteriores do setup.bash.

## 6. Executar — dois terminais

### Terminal 1: cenário, dentro da Distrobox

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
cd ~/RSDIS_ROS2_Mars_Simulation
source install/local_setup.bash
ros2 launch spaceros_gz_demos mars.launch.xml
```

Carrega no botão ▶. Confirma que o tempo da simulação avança.

### Terminal 2: teleop, dentro da mesma Distrobox

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
cd ~/RSDIS_ROS2_Mars_Simulation
source install/local_setup.bash
ros2 run perseverance_control perseverance_node
```

Fecha o teleop antigo e o nó de avanço contínuo. Só deve existir um programa
a decidir a velocidade. O arranque envia zero; aguarda a descoberta ROS antes
de começar a conduzir. Carrega em W e depois em Espaço.

## 7. Explicação do código, pela ordem de leitura

### Constantes

LINEAR_SPEED e ANGULAR_SPEED definem a velocidade pedida. PUBLISH_HZ=20
significa um envio aproximadamente a cada 50 ms. KEY_TIMEOUT_S=0.8 limita
o tempo sem input. O 10 de create_publisher é profundidade da fila, não Hz.

### update_command(key, cmd)

Recebe uma tecla e modifica a mensagem existente. O & significa referência:
não se altera uma cópia independente. Primeiro substitui cmd por Twist{}
(todos os componentes a zero), depois escolhe uma componente no switch.
Isto impede que uma rotação mantenha por engano o avanço anterior.

Twist contém dois vetores: linear e angular, cada um com x, y, z.
Neste controlador diferencial usamos linear.x e angular.z. Pedir uma
velocidade não prova que o rover a atingiu; isso exige feedback de sensores.

### Keyboard: porquê uma pequena classe?

É preciso guardar a configuração original do terminal, alterá-la e repô-la
mesmo quando ocorre uma exceção. O construtor prepara o terminal; o destrutor
repõe a configuração. Essa associação entre objeto e recurso chama-se RAII.
As cópias são proibidas para não haver dois objetos a restaurar o mesmo recurso.

ICANON desligado permite receber caracteres sem Enter. ECHO desligado impede
que as letras apareçam no ecrã. VMIN=0/VTIME=0 e poll com timeout zero permitem
consultar o teclado sem bloquear o ciclo. ISIG é preservado para Ctrl+C gerar
um sinal. IXON é removido para Ctrl+S não congelar a saída do terminal.

read_key() devolve true se obteve um carácter; false se não há input.
Um terminal desligado ou erro de leitura gera uma exceção, conduzindo à saída
com tentativa de paragem. O nó deve correr num terminal interativo em primeiro
plano, não numa consola Output do editor nem com stdin redirecionado.

### O ciclo principal

1. Lê até 64 caracteres disponíveis, sem esperar.
2. Interpreta teclas normais ou deteta um pedido de saída.
3. Mede o tempo desde a última tecla.
4. Coloca a velocidade a zero se o tempo expirou ou foi pedida saída.
5. Publica a mensagem.
6. Processa eventos ROS com spin_some().
7. Espera o próximo ciclo com WallRate.

O limite de 64 evita que uma colagem enorme monopolize uma iteração. Não
coles texto no terminal de condução: letras desse texto seriam comandos.
O relógio steady_clock mede intervalos sem saltos do relógio civil. O timeout
e a frequência usam tempo real do computador, mesmo se o Gazebo estiver pausado.
Não se adiciona spin() depois do while: spin_some() já processa os eventos.

### Saída com Q ou Ctrl+C

Q define stop_requested. Ctrl+C produz SIGINT; a função request_stop()
apenas altera a mesma flag do tipo sig_atomic_t. O código ROS não corre
dentro do handler do sinal.

SignalHandlerOptions::None desativa os handlers automáticos de rclcpp.
Assim o contexto ROS permanece ativo enquanto publicamos cinco mensagens
zero, separadas por 50 ms, antes de shutdown(). SIGTERM, SIGHUP e SIGQUIT
também pedem esta saída. O terminal é restaurado ao sair do bloco try.

Estas publicações são uma tentativa de paragem, não uma confirmação de que
o rover parou. SIGKILL, falha do processo, rede ou ponte podem impedir o envio.
Um sistema real precisa também de um timeout de comandos no controlador do
robô. Não confundas este timeout local do teclado com proteção no recetor.

## 8. Testes no teu ambiente

1. Arranca o teleop: o rover deve ficar parado.
2. W/S devem produzir avanço/recuo.
3. A/D devem produzir rotação em sentidos opostos.
4. W seguido de A deve remover a velocidade linear.
5. Espaço deve publicar zero no ciclo seguinte.
6. Larga a tecla: após aproximadamente 0,8 s deve publicar zero.
7. Q e Ctrl+C devem tentar parar, sair e devolver o terminal ao modo normal.
8. Volta a arrancar: não deve reutilizar o último movimento.

Para observar as mensagens, num terceiro terminal Jazzy no domínio 30:

```bash
ros2 topic echo /perseverance/cmd_vel
ros2 topic info /perseverance/cmd_vel --verbose
```

Executa os comandos separadamente; Ctrl+C termina o echo.
Deve existir um publisher chamado perseverance_control. A ponte bidirecional
pode aparecer também como publisher: isso não é, por si só, um erro.

No lado do Gazebo:

```bash
gz topic -e -t /model/nasa_perseverance/cmd_vel
```

Os nomes ROS e Gazebo são diferentes por causa do remapeamento da ponte.
Seleciona o terminal teleop para carregar nas teclas e observa o outro.

### Se não andar

- Confirma que o Gazebo não está pausado.
- Confirma que o nó foi recompilado e que o terminal teleop tem foco.
- Confirma ROS_DOMAIN_ID=30 antes de arrancar cada programa.
- Se só a ponte aparece como publisher, verifica se executaste a versão nova.
- Se o pacote não for encontrado, carrega install/local_setup.bash.
- Se o compilador disser que o ficheiro não existe, verifica nome e caminho.
- Se só o braço mexer, observa cmd_vel e odometria; não assumas que a tecla
  controla o braço. Este nó só publica velocidade da base.

### Paragem de recurso já usada neste projeto

Termina primeiro todos os programas que publicam movimento; depois:

```bash
gz topic -t /model/nasa_perseverance/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}'
```

Se o terminal ficar sem eco após uma interrupção forçada, executa stty sane.

## 9. O que foi validado nesta entrega

O ambiente de preparação não tem ROS 2 nem Gazebo: não foi possível executar
colcon nem testar o rover aqui. A leitura do terminal foi compilada com g++
e testada num pseudoterminal Linux, incluindo leitura sem Enter, retorno sem
input e restauro das configurações. O package.xml foi verificado como XML.
A compilação completa e os testes do ponto 8 devem ser feitos na Distrobox.

## 10. Para estudar a seguir

1. Altera a velocidade e explica a diferença entre m/s e rad/s.
2. Acrescenta uma tecla de curva: avanço e rotação na mesma mensagem.
3. Recebe /perseverance/odometry e compara pedido com movimento observado.
4. Só depois separa controlo manual de navegação autónoma.

Referências:
- [Tutoriais ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Tutorials.html)
- [Código/documentação de inicialização rclcpp Jazzy](https://api.nav2.org/jazzy/html/utilities_8cpp_source.html)
- [Terminal Linux: termios](https://man7.org/linux/man-pages/man3/termios.3.html)
- [Cenário original](https://github.com/david-dorf/spaceros_gz_demos)
