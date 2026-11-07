# RSDIS — Referências técnicas e bibliográficas

**Projeto:** Simulação de uma missão robótica em Marte (ROS 2 Jazzy, Gazebo, Perseverance e Mars Pathfinder)  
**Atualizado em:** 7 de novembro de 2026

> Lista de referências organizada por tema para consulta durante o desenvolvimento e preparação do relatório. Os links remetem para a documentação e recursos originais. Confirma a versão da documentação consultada e as condições de licenciamento dos modelos 3D antes da entrega.

## 1. ROS 2 Jazzy e desenvolvimento em C++

| Referência | Aplicação no projeto |
|---|---|
| [ROS 2 Jazzy — Documentação oficial](https://docs.ros.org/en/jazzy/) | Referência geral da plataforma. |
| [rclcpp — API C++](https://docs.ros.org/en/jazzy/p/rclcpp/) | Implementação dos nós ROS 2. |
| [Publishers e subscribers em C++](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) | Comunicação entre nós. |
| [Parâmetros em classes C++](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) | Configuração da navegação sem recompilação. |
| [Action Server e Client em C++](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html) | Objetivos de navegação e missões. |
| [Criação de packages ROS 2](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) | Organização modular do workspace. |

## 2. Gazebo, SDF e integração com ROS 2

| Referência | Aplicação no projeto |
|---|---|
| [Gazebo Harmonic — Documentação oficial](https://gazebosim.org/docs/harmonic/) | Simulador. |
| [Gazebo — SDF Worlds](https://gazebosim.org/docs/harmonic/sdf_worlds/) | Construção e edição de `mars.sdf`. |
| [Gazebo — Fuel Insert](https://gazebosim.org/docs/harmonic/fuel_insert/) | Inserção de modelos. |
| [SDFormat — Especificação oficial](https://sdformat.org/spec/) | Referência para ficheiros `.sdf`. |
| [SDFormat 1.9 — World](https://sdformat.org/spec?ver=1.9&elem=world) | Configuração do mundo simulado. |
| [SDFormat 1.9 — Link](https://sdformat.org/spec?ver=1.9&elem=link) | Elementos físicos e visuais dos modelos. |
| [SDFormat 1.9 — Material](https://sdformat.org/spec?ver=1.9&elem=material) | Aspeto das superfícies. |
| [SDFormat 1.9 — Include](https://sdformat.org/spec?ver=1.9&elem=include) | Inclusão do Mars Pathfinder no mundo. |
| [SDFormat 1.9 — Geometry](https://sdformat.org/spec?ver=1.9&elem=geometry) | Meshes e geometrias de colisão. |
| [Gazebo — ROS 2 Integration](https://gazebosim.org/docs/harmonic/ros2_integration/) | Ligação entre simulador e ROS. |
| [ros_gz_bridge](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/) | Bridge de mensagens. |
| [Gazebo — Finding Resources](https://gazebosim.org/api/sim/9/resources.html) | Resolução de caminhos de modelos e texturas. |

## 3. Sensores, câmaras e deteção de obstáculos

| Referência | Aplicação no projeto |
|---|---|
| [sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) | Nuvens de pontos da câmara de profundidade. |
| [sensor_msgs/msg/Image](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Image.html) | Imagem da câmara. |
| [sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/CameraInfo.html) | Calibração e geometria da câmara. |
| [sensor_msgs/msg/LaserScan](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/LaserScan.html) | Varreduras de sensores LiDAR. |
| [Gazebo — Sensors](https://gazebosim.org/docs/harmonic/sensors/) | Sensores simulados. |
| [Point Cloud Library — Documentação](https://pointclouds.org/documentation/) | Processamento de dados 3D. |
| [Point Cloud Library — Tutoriais](https://pointclouds.org/documentation/tutorials/) | Filtros e algoritmos para nuvens de pontos. |

## 4. Navegação autónoma e planeamento de trajetórias

| Referência | Aplicação no projeto |
|---|---|
| [Hart, Nilsson e Raphael (1968) — Artigo original do A*](https://doi.org/10.1109/TSSC.1968.300136) | Fundamentação científica do planeamento. |
| [Navigation2 (Nav2) — Documentação](https://docs.nav2.org/) | Arquitetura de navegação autónoma. |
| [Nav2 — Navigation Plugins](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/) | Planeadores e controladores. |
| [Nav2 — Smac Planner](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/) | Referência de implementações A*. |
| [Nav2 Jazzy — Smac Planner 2D](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/) | A* em grelha 2D. |
| [nav_msgs/msg/Path](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Path.html) | Trajetória calculada. |
| [nav_msgs/msg/Odometry](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/Odometry.html) | Estimativa de posição e movimento. |
| [geometry_msgs/msg/Twist](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/Twist.html) | Comandos de velocidade para o rover. |

## 5. Mapeamento, grelhas e transitabilidade

| Referência | Aplicação no projeto |
|---|---|
| [nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/jazzy/p/nav_msgs/msg/OccupancyGrid.html) | Mapa de ocupação 2D. |
| [Nav2 — Costmap 2D](https://docs.nav2.org/configuration/packages/configuring-costmaps.html) | Custos de navegação e obstáculos. |
| [ANYbotics — Grid Map](https://github.com/ANYbotics/grid_map) | Mapas de elevação e características do terreno. |
| [OctoMap](https://octomap.github.io/) | Representação probabilística do espaço 3D. |
| [Point Cloud Library — Tutoriais](https://pointclouds.org/documentation/tutorials/) | Conversão e filtragem de dados de profundidade. |

## 6. Missões e veículos espaciais da NASA

| Referência | Aplicação no relatório |
|---|---|
| [NASA — Mars 2020 / Perseverance](https://science.nasa.gov/mission/mars-2020-perseverance/) | Contextualização e características do rover. |
| [NASA — Mars Pathfinder](https://science.nasa.gov/mission/mars-pathfinder/) | Ponto de interesse e missão histórica. |
| [NASA — Ingenuity Mars Helicopter](https://science.nasa.gov/mission/mars-2020-perseverance/ingenuity-mars-helicopter/) | Referência para reconhecimento aéreo. |
| [NASA — Mars Exploration](https://science.nasa.gov/mars/) | Contexto geral da exploração marciana. |
| [NASA — Jet Propulsion Laboratory](https://www.jpl.nasa.gov/) | Engenharia e missões robóticas. |

## 7. Modelos 3D e recursos gráficos

| Referência | Aplicação no projeto |
|---|---|
| [3D Warehouse — Mars Pathfinder Lander HIGH Detail](https://3dwarehouse.sketchup.com/model/u369e1cb8-af6e-46dc-9d4d-57a919f5b2ca/Mars-Pathfinder-Lander-HIGH-Detail) | Modelo COLLADA utilizado como referência visual. |
| [Gazebo Fuel — Biblioteca de modelos](https://app.gazebosim.org/fuel/models) | Outros modelos compatíveis com Gazebo. |
| [Blender — Manual](https://docs.blender.org/manual/en/latest/) | Conversão e edição de meshes. |
| [Khronos — COLLADA](https://www.khronos.org/collada/) | Especificação do formato `.dae`. |
| [NASA — 3D Resources](https://nasa3d.arc.nasa.gov/) | Recursos visuais de missões espaciais. |
| [Smithsonian — Mars Pathfinder Lander Model](https://www.si.edu/object/model-mars-pathfinder-lander%3Anasm_A20020299000) | Referência visual e histórica. |
| [Pinshape — Mars Pathfinder and Sojourner](https://pinshape.com/items/33847-3d-printed-mars-pathfinder-and-sojourner) | Alternativa para visualização/impressão; verificar licença antes de reutilizar. |

## 8. Dez referências prioritárias para o relatório

1. [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — infraestrutura de software.
2. [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/) — ambiente de simulação.
3. [SDFormat](https://sdformat.org/spec/) — descrição de mundos e modelos.
4. [Gazebo / ROS 2 Integration](https://gazebosim.org/docs/harmonic/ros2_integration/) — integração dos sistemas.
5. [NASA Perseverance](https://science.nasa.gov/mission/mars-2020-perseverance/) — fundamentação da missão.
6. [NASA Mars Pathfinder](https://science.nasa.gov/mission/mars-pathfinder/) — alvo da navegação.
7. [PointCloud2](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/PointCloud2.html) — deteção de obstáculos.
8. [Hart et al. (1968), A*](https://doi.org/10.1109/TSSC.1968.300136) — algoritmo de planeamento.
9. [Nav2 Smac Planner](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/smac/) — solução de referência.
10. [ANYbotics Grid Map](https://github.com/ANYbotics/grid_map) — representação da elevação do terreno.

## 9. Referência bibliográfica científica pronta a adaptar

Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. **IEEE Transactions on Systems Science and Cybernetics, 4**(2), 100–107. https://doi.org/10.1109/TSSC.1968.300136

---

**Notas para a entrega:**

- A lista é uma compilação de recursos técnicos, **não** uma bibliografia final formatada integralmente em IEEE ou APA.
- Cita no relatório apenas as fontes efetivamente consultadas e utilizadas.
- Para recursos online, regista a data de consulta e, quando aplicável, a versão da documentação.
- Para o modelo 3D descarregado do 3D Warehouse, identifica o autor original e confirma a licença de utilização e redistribuição.
- Alguns endereços de documentação podem mudar entre versões; os links estão organizados para consulta, não constituem garantia de disponibilidade permanente.
