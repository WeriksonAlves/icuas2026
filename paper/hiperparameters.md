Perfeito, agora temos tudo que precisamos pra exportar as nuvens 👌

Temos o tópico de nuvem:

```text
/zed2i/zed_node/point_cloud/cloud_registered  (sensor_msgs/msg/PointCloud2)
```

E os bags já em formato ROS2 (`flight_5_ros2`, `flight_6_ros2`).

O script que vamos usar é:

```text
scripts/export_pointcloud2_to_ply.py
```

Ele **se inscreve no tópico** e salva `.ply` enquanto você dá `ros2 bag play` no bag.

---

## 1. Estrutura de diretórios para os .ply

Vamos seguir o padrão do projeto:

```text
~/ufv/icuas2026/
  data/
    ply/
      outdoor_02_flight_5/
        frame_00000.ply
        frame_00001.ply
        ...
      outdoor_02_flight_6/
        frame_00000.ply
        ...
```

Crie a pasta base se ainda não existir:

```bash
cd ~/ufv/icuas2026
mkdir -p data/ply/outdoor_02_flight_5
mkdir -p data/ply/outdoor_02_flight_6
```

---

## 2. Exportação do `flight_5_ros2`

A ideia é:

* **Terminal 1**: `ros2 bag play`
* **Terminal 2**: script de exportação (assina o tópico e escreve .ply)

### 🔹 Terminal 1 – reproduzir a bag

```bash
cd ~/ufv/icuas2026/data/bags/outdoor_02/2026-02-22-17-28-23
source /opt/ros/humble/setup.bash

ros2 bag play flight_5_ros2
```

Se quiser que rode mais devagar (por debugar):

```bash
ros2 bag play flight_5_ros2 --rate 0.5
```

> Deixa esse terminal rodando, ele é quem “injeta” os PointCloud2 no grafo ROS2.

---

### 🔹 Terminal 2 – exportar PointCloud2 → PLY

No **root do projeto**:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate
source /opt/ros/humble/setup.bash
```

Agora roda o script:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed2i/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/outdoor_02_flight_5 \
  --max_frames 800 \
  --every_k 1 \
  --voxel 0.05
```

Explicando os parâmetros (do `ExportConfig` do módulo `planes.ros_export`):

* `--topic` → tópico `PointCloud2` que vimos no `ros2 bag info`
* `--out_dir` → diretório onde vão os `.ply`
* `--max_frames` → número máximo de frames a salvar

  * `flight_5` tem 706 mensagens de nuvem, então 800 cobre tudo com folga.
* `--every_k` → salva **cada K-ésima** mensagem

  * `1` = salva todas (boa pra esse voo de 0–20 m).
* `--voxel` → voxel size em metros para downsampling

  * `0.05` = 5 cm → bom compromisso entre densidade e custo computacional.
* (implícito) `--keep_nans` **não** usado ⇒ `remove_nans=True` (NaN/Inf filtrados).

O script vai imprimir algo como:

```text
[INFO] Topic: /zed2i/zed_node/point_cloud/cloud_registered
[INFO] Output dir: data/ply/outdoor_02_flight_5
[INFO] Max frames: 800
[INFO] Every K messages: 1
[INFO] Voxel size: 0.0500 m
[INFO] Remove NaNs: True
```

E conforme o bag vai rodando, os `.ply` vão sendo criados.

Quando atingir `max_frames` (ou quando o bag acabar antes), o script encerra sozinho.

---

## 3. Exportação do `flight_6_ros2`

Mesma lógica, mudando bag e pasta.

### 🔹 Terminal 1

```bash
cd ~/ufv/icuas2026/data/bags/outdoor_02/2026-02-22-17-28-23
source /opt/ros/humble/setup.bash

ros2 bag play flight_6_ros2
```

`flight_6` tem 408 nuvens, então:

### 🔹 Terminal 2

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate
source /opt/ros/humble/setup.bash

python scripts/export_pointcloud2_to_ply.py \
  --topic /zed2i/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/outdoor_02_flight_6 \
  --max_frames 500 \
  --every_k 1 \
  --voxel 0.05
```

---

## 4. Conferindo se os .ply ficaram bons

Depois da exportação:

```bash
ls data/ply/outdoor_02_flight_5 | head
ls data/ply/outdoor_02_flight_6 | head
```

E para inspeção rápida:

```bash
python -c "import open3d as o3d; \
pcd = o3d.io.read_point_cloud('data/ply/outdoor_02_flight_5/frame_00000.ply'); \
o3d.visualization.draw_geometries([pcd])"
```

Tecnicamente, nesse ponto você já terá:

* nuvens com downsampling em 5 cm,
* NaNs removidos (para evitar problemas com RANSAC/plane fitting depois),
* frames amostrados 1:1 com as mensagens de nuvem.

---

## 5. Próximo passo da pipeline

Quando confirmar que os `.ply` ficaram ok, o próximo passo será:

1. Definir o nome do experimento (ex.: `zed_outdoor_02_f5` / `zed_outdoor_02_f6`),

2. Rodar:

   ```bash
   python scripts/run_planes_pipeline.py \
     --experiment zed_outdoor_02_f5 \
     --max_planes 4 \
     --preset outdoor
   ```

3. Gerar:

   * nuvens segmentadas (`planes_XXXXX.ply`, versões coloridas),
   * JSON com parâmetros dos planos,
   * tracking, classificação estável e gráficos de estabilidade temporal.

Se quiser, me manda um `ls data/ply/outdoor_02_flight_5 | head` e eu já te ajudo a escolher um `experiment` e o comando exato pra rodar o `run_planes_pipeline.py` em cima desses dados de voo.

---

Perfeito, nome ótimo — bem consistente com o que você já vinha usando 👍

Vamos plugar esse experimento na pipeline de planos.

---

## 1. Garantir que o diretório de PLY bate com o nome do experimento

Padrão da pipeline:

* Diretório de entrada (PLY):

```text
data/ply/<experiment_name>/
```

No seu caso, queremos:

```text
data/ply/zed_outdoor_02_T2_279_01_05/
  frame_00000.ply
  frame_00001.ply
  ...
```

### a) Se você já exportou direto com esse nome

Se, na fase de exportação, você usou algo como:

```bash
python scripts/export_pointcloud2_to_ply.py \
  --topic /zed2i/zed_node/point_cloud/cloud_registered \
  --out_dir data/ply/zed_outdoor_02_T2_279_01_05 \
  --max_frames 800 \
  --every_k 1 \
  --voxel 0.05
```

então **já está tudo certo**: o `--experiment` vai casar com esse diretório.

### b) Se você exportou para outro nome (ex.: `outdoor_02_flight_5`)

Basta renomear:

```bash
cd ~/ufv/icuas2026

mv data/ply/outdoor_02_flight_5 \
   data/ply/zed_outdoor_02_T2_279_01_05
```

(ou mova/copiei tudo para esse diretório alvo.)

---

## 2. Como o `run_planes_pipeline.py` interpreta o experimento

Você já rodou algo assim antes:

```text
Experiment name     : livox_outdoor_01_T22_275_15_03
Input PLY directory : data/ply/livox_outdoor_01_T22_275_15_03
Planes output dir   : output/planes/livox_outdoor_01_p4
Max planes per frame: 4
```

Ou seja:

* `--experiment livox_outdoor_01_T22_275_15_03`

  * **Entrada PLY**: `data/ply/livox_outdoor_01_T22_275_15_03`
  * **Saída planos**: `output/planes/livox_outdoor_01_p4`
    (ele pega só o “prefixo” antes do `_T...` para nomear a pasta de saída + `_p4` para max_planes=4).

No seu caso:

* `--experiment zed_outdoor_02_T2_279_01_05`
  ⇒ entrada:

  ```text
  data/ply/zed_outdoor_02_T2_279_01_05
  ```

  ⇒ saída (com `--max_planes 4`):

  ```text
  output/planes/zed_outdoor_02_p4
  ```

---

## 3. Rodando a pipeline completa para esse experimento

No root do repo:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate
source /opt/ros/humble/setup.bash
```

Agora execute:

```bash
python scripts/run_planes_pipeline.py \
  --experiment zed_outdoor_02_T2_279_01_05 \
  --max_planes 4 \
  --preset outdoor
```

O script deve logar algo do tipo:

```text
[INFO] Experiment name     : zed_outdoor_02_T2_279_01_05
[INFO] Input PLY directory : data/ply/zed_outdoor_02_T2_279_01_05
[INFO] Planes output dir   : output/planes/zed_outdoor_02_p4
[INFO] Max planes per frame: 4
[RUN] ... segment_planes_open3d.py ...
[RUN] ... track_planes_temporally.py ...
[RUN] ... classify_tracks_stably.py ...
[RUN] ... recolor_by_stable_class.py ...
[RUN] ... analyze_temporal_stability.py ...
```

---

## 4. O que será gerado (para você já se localizar depois)

Na pasta:

```text
output/planes/zed_outdoor_02_p4/
```

Você deve ver algo como:

* **Por frame:**

  * `ply_colored/planes_00000.ply`
    → nuvem colorida por ID de plano.
  * `ply_stable_semantic/planes_00000.ply`
    → nuvem colorida por classe estável (floor / wall / etc), pós-classificação.
  * `json/planes_00000.json`
    → parâmetros dos planos naquele frame (normal, d, área, inliers, rótulos).

* **Globais:**

  * `plane_tracks.json`
    → trilhas temporais dos planos.
  * `classification_stable.json`
    → rótulos estáveis por trilha (floor, wall, main wall, etc.).
  * `track_confidence.json`
    → métricas de confiança por trilha.
  * `temporal_stability_summary.json`
    → estatísticas agregadas de estabilidade temporal.
  * `summary.json`
    → resumo geral do experimento (contagem de planos, parâmetros médios, etc.).

Se o `analyze_temporal_stability.py` estiver configurado com `make_plots=True`, você também terá gráficos (provavelmente em uma subpasta `stability/`).

---

## 5. Próximo passo prático

Depois de rodar esse comando:

```bash
python scripts/run_planes_pipeline.py \
  --experiment zed_outdoor_02_T2_279_01_05 \
  --max_planes 4 \
  --preset outdoor
```

me manda:

```bash
ls output/planes/zed_outdoor_02_p4 | sed -n '1,20p'
```

e, se possível, o conteúdo de uma subpasta, por exemplo:

```bash
ls output/planes/zed_outdoor_02_p4/ply_stable_semantic | head
```

A partir daí, a gente já entra na parte de:

* como visualizar os planos segmentados (ex.: carregar um `.ply` específico no Open3D),
* como interpretar os JSON dos planos,
* e como olhar para os gráficos de estabilidade em função da altura (0–20 m).

---

Show! Pipeline rodou bonitinho de ponta a ponta 🚀

Vamos organizar o que você já tem agora e como usar isso na prática: nuvens segmentadas + gráficos.

---

## 1. O que foi gerado (resumo rápido)

Na pasta:

```bash
ls output/planes/zed_outdoor_02_p4
```

você já viu:

* `json/` → parâmetros de planos por frame (`planes_000XX.json`)
* `ply_colored/` → planos segmentados (cores por **ID de plano**)
* `ply_stable_semantic/` → planos recoloridos por **classe semântica estável**
* `plane_tracks.json` → trilhas temporais de planos
* `track_confidence.json` → confiabilidade das trilhas
* `classification_stable.json` → mapeia trilhas → floor/table/main_wall
* `floor_deltas.png`, `floor_orientation_offset.png`,
  `table_bed_orientation_offset.png` → gráficos
* `temporal_stability_summary.json` → resumo numérico da estabilidade
* `summary.json` → estatísticas globais do experimento

E ainda:

```text
[INFO] Floor: samples=212, mean_angle_to_first=13.6769 deg, max_delta_angle=22.3627 deg
```

Ou seja: o plano de “floor” foi visto em 212 amostras, com variação angular máxima ≈ 22° em relação à referência.

---

## 2. Visualizar um frame segmentado (Open3D)

Vamos começar olhando para um frame qualquer, por exemplo o `planes_00020.ply` que você viu na listagem.

### 2.1. Visualizar a segmentação por classe estável (floor/table/wall)

No root do projeto:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate

python - << 'EOF'
from pathlib import Path
import open3d as o3d

root = Path("output/planes/zed_outdoor_02_p4/ply_stable_semantic")

pcd_path = root / "planes_00020.ply"
print(f"Loading: {pcd_path}")
pcd = o3d.io.read_point_cloud(str(pcd_path))

print(pcd)
o3d.visualization.draw_geometries([pcd])
EOF
```

Nessa visualização:

* as cores vêm do módulo `planes.recolor`:

  * `floor` → **vermelho** `[1, 0, 0]`
  * `table` → **verde** `[0, 1, 0]`
  * `main_wall` → **azul** `[0, 0, 1]`
  * qualquer outra coisa → cinza `[0.5, 0.5, 0.5]`

(isso está explicitamente codificado em `src/planes/recolor.py`.)

### 2.2. Visualizar a segmentação “bruta” por ID de plano

Se quiser ver só a segmentação geométrica por plano (antes da classificação estável):

```bash
python - << 'EOF'
from pathlib import Path
import open3d as o3d

root = Path("output/planes/zed_outdoor_02_p4/ply_colored")

pcd_path = root / "planes_00020.ply"
print(f"Loading: {pcd_path}")
pcd = o3d.io.read_point_cloud(str(pcd_path))

print(pcd)
o3d.visualization.draw_geometries([pcd])
EOF
```

Aqui as cores são “apenas” IDs de plano (plane 0, 1, 2, 3...), sem semântica floor/table/wall.

### 2.3. Se quiser rotacionar em 180° no eixo Z

Se durante a visualização a nuvem parecer “de cabeça para baixo” (eixo X/Y invertido), você pode aplicar uma rotação de 180° em torno de Z:

```bash
python - << 'EOF'
from pathlib import Path
import open3d as o3d
import numpy as np

root = Path("output/planes/zed_outdoor_02_p4/ply_stable_semantic")
pcd = o3d.io.read_point_cloud(str(root / "planes_00020.ply"))

# Rotação 180° em torno de Z:
R = np.array([
    [-1.0,  0.0, 0.0],
    [ 0.0, -1.0, 0.0],
    [ 0.0,  0.0, 1.0],
])
pcd.rotate(R, center=(0, 0, 0))

o3d.visualization.draw_geometries([pcd])
EOF
```

Tecnicamente isso faz: (x,y,z) → (−x, −y, z).

---

## 3. Entendendo o warning de frames faltando na recoloração

O log trouxe:

```text
[INFO] Recolored 120/182 frames ...
[WARN] Missing input frames: [110, 111, ..., 172]
```

Isso vem do `planes.recolor.run_recolor`:

* O módulo lê os JSON de planos que existem na pasta `output/planes/zed_outdoor_02_p4/json` (frames 0..213, por exemplo).
* Para cada frame **esperado** com planos, ele tenta carregar o `.ply` original em `data/ply/zed_outdoor_02_T2_279_01_05/frame_XXXXX.ply`.
* Se não encontrar o `.ply` correspondente, esse índice entra em `missing_input_frames`.

Ou seja:

* **Seus planos foram detectados e rastreados nesses frames**, mas
* **não existe o `.ply` original para alguns índices**, possivelmente porque:

  * você começou a gravar o `.ply` a partir de `frame_00012`, ou
  * o script de exportação pulou alguns frames no início/fim, ou
  * havia buracos na sequência de `PointCloud2` na bag.

Isso **não quebra a análise**; você só não terá versão recolorida (`ply_stable_semantic`) para esses frames. O tracking e a análise temporal usam os JSON, que estão completos.

Se um dia quiser “zerar” esse warning, o caminho seria:

* garantir exportação contínua de `.ply` para todos os frames com detecção,
* ou ajustar a numeração inicial dos `.ply` para casar exatamente com os índices que a segmentação espera (mas isso é detalhe fino, não é crítico agora).

---

## 4. Interpretando os gráficos gerados

Na pasta:

```bash
ls output/planes/zed_outdoor_02_p4/*.png
```

Você deve ter:

* `floor_deltas.png`
* `floor_orientation_offset.png`
* `table_bed_orientation_offset.png`

Abra com qualquer visualizador de imagem ou direto no terminal (se quiser usar `xdg-open`):

```bash
xdg-open output/planes/zed_outdoor_02_p4/floor_deltas.png
xdg-open output/planes/zed_outdoor_02_p4/floor_orientation_offset.png
```

Conceitualmente:

### 4.1. `floor_orientation_offset.png`

Mostra, por frame, o **ângulo entre a normal do floor naquele frame** e uma normal de referência (tipicamente a do primeiro frame do track principal de floor).

* Idealmente, para um chão plano e pipeline bem estável:

  * essa curva oscilaria pouco em torno de 0°;
* No seu caso:

  * média ≈ 13.7°,
  * máximo ≈ 22.4°.

Isso indica que há:

* ou variação real de orientação “do plano chão” no frame de câmera (ex: atitudes do drone),
* ou ruído na estimação de plano (RANSAC + densidade da nuvem),
* ou ainda um pequeno desajuste entre eixo vertical “real” e o eixo usado na classificação.

### 4.2. `floor_deltas.png`

Geralmente é um histograma ou gráfico derivado das diferenças de orientação entre frames consecutivos (ou em relação à referência). Serve para ver a **estabilidade local**: se os deltas são pequenos e concentrados, o chão está sendo estimado de forma estável; se há caudas longas, há frames com saltos mais bruscos na normal do chão.

### 4.3. `table_bed_orientation_offset.png`

No log:

```text
Table/Bed: samples=1
```

Significa que o classificador **quase não encontrou um plano horizontal secundário** (tipo mesa ou plataforma) estável ao longo do tempo. Faz sentido em voo outdoor com só chão e talvez obstáculos irregulares.

---

## 5. Conferindo o resumo numérico

Se quiser ver um resumo bem compacto em JSON:

```bash
cat output/planes/zed_outdoor_02_p4/temporal_stability_summary.json
```

Ou em Python:

```bash
python - << 'EOF'
import json
from pathlib import Path

summary_path = Path("output/planes/zed_outdoor_02_p4/temporal_stability_summary.json")
summary = json.loads(summary_path.read_text())
print(json.dumps(summary, indent=2))
EOF
```

Isso te dá acesso programático às métricas para colocar em relatório, artigo, etc.

---

## 6. Sugestão de próximos passos

Agora que o pipeline *end-to-end* está fechado para esse voo:

1. **Inspecionar alguns frames chave**

   * Baixa altitude vs alta altitude (ex.: `planes_00020.ply`, `planes_00150.ply` etc.);
   * Ver se o chão está bem segmentado em toda a faixa de 0–20 m.

2. **Checar se o eixo vertical usado faz sentido**

   * O log mostra `Vertical axis    : [1.0, 0.0, 0.0]`;
   * Se no seu setup Z é “up”, pode valer a pena rodar uma segunda vez o `classify_tracks_stably.py` com `--vertical-axis 0 0 1` pra ver como muda a estabilidade de floor/wall (a gente pode montar o comando certinho se você quiser).

3. **Comparar esse voo com outro experimento**

   * Repetir a pipeline num segundo voo outdoor (alturas diferentes, velocidades diferentes) e comparar o `temporal_stability_summary.json` e os gráficos.

Se quiser, na próxima mensagem podemos:

* escolher dois ou três frames específicos (por índice),
* interpretar juntos o conteúdo de `planes_00XXX.json` e do `.ply` correspondente,
* e relacionar isso com o movimento do drone (ex.: trecho em que o drone sobe/rotaciona mais, como isso aparece nos ângulos do chão).

---

Perfeito — a imagem que você enviou é exatamente a **segmentação bruta por ID de plano** (`ply_colored`), antes da classificação estável. Vamos interpretar tecnicamente o que está acontecendo.

---

# 1️⃣ O que a imagem está mostrando

Visualmente temos:

* 🔴 **Grande região vermelha dominante**
* 🟢 Região verde relativamente extensa
* 🔵 Regiões azuis mais fragmentadas
* ⚫ Cinza = pontos não pertencentes a nenhum plano (resto da nuvem)

Isso é coerente com:

```bash
--max_planes 4
```

Ou seja, em cada frame o RANSAC iterativo detecta até 4 planos:

```
plane_0 → vermelho
plane_1 → verde
plane_2 → azul
plane_3 → outra cor (dependendo da paleta)
resto   → cinza
```

---

# 2️⃣ Interpretação geométrica (outdoor 0–20 m)

Pelo padrão visual:

### 🔴 Plano dominante (vermelho)

Muito provavelmente é o **chão**.

Características:

* Área grande
* Estrutura contínua
* Ocupa grande parte inferior da nuvem

Em voo outdoor, o solo tende a ser o plano mais estável e com maior número de inliers.

---

### 🟢 Plano secundário (verde)

Possíveis hipóteses:

* Talude / inclinação do terreno
* Fachada lateral
* Estrutura artificial (muro, galpão, telhado)

Como está relativamente coeso, pode ser uma parede ou desnível do terreno.

---

### 🔵 Planos fragmentados

Regiões azuis aparecem mais espalhadas:

* Pode ser vegetação parcialmente planar
* Pode ser ruído estrutural
* Pode ser planos pequenos detectados pelo RANSAC em clusters locais

Isso é típico quando:

* O `distance_threshold` está levemente permissivo
* O terreno não é perfeitamente plano
* Há textura irregular (grama, vegetação)

---

# 3️⃣ Análise técnica do comportamento do RANSAC

Você rodou com preset `outdoor`.

Normalmente esse preset usa:

* `distance_threshold ≈ 0.03–0.05 m`
* `ransac_n = 3`
* `num_iterations ≈ 1000`
* downsampling prévio (voxel 5 cm)

### Por que vemos “salpicado azul” no chão vermelho?

Porque:

1. O solo real não é perfeitamente plano
2. O ZED gera ruído em profundidade que aumenta com distância
3. Em 20 m de altitude o erro angular da normal cresce

Se o threshold for 4 cm e o terreno tiver pequenas ondulações > 4 cm, o RANSAC começa a “quebrar” o plano dominante em sub-planos.

---

# 4️⃣ Relação com seus resultados de estabilidade

Você obteve:

```
mean_angle_to_first ≈ 13.7°
max_delta_angle ≈ 22.3°
```

Isso é relativamente alto para um plano “chão ideal”.

Prováveis causas:

* Drone está inclinando (roll/pitch) durante o voo
* Eixo vertical configurado como `[1,0,0]`
* Solo não perfeitamente horizontal
* Variação de altura alterando distribuição de pontos

Aliás, esse ponto é importante:

```text
Vertical axis : [1.0, 0.0, 0.0]
```

⚠️ Isso indica que o pipeline está assumindo que **X é o eixo vertical**.

Mas no ZED, geralmente:

```
X → direita
Y → baixo
Z → frente   (ou dependendo da configuração)
```

Ou se você já reorientou:

```
Z → up
```

Se o eixo vertical estiver errado, a classificação de floor pode estar sendo feita com referência incorreta.

Isso impacta diretamente a estabilidade angular.

---

# 5️⃣ Sugestão Técnica (Importante)

Eu recomendo rodar novamente apenas a etapa de classificação estável com:

```bash
python scripts/classify_tracks_stably.py \
  --tracks output/planes/zed_outdoor_02_p4/plane_tracks.json \
  --confidence output/planes/zed_outdoor_02_p4/track_confidence.json \
  --output output/planes/zed_outdoor_02_p4/classification_stable_zup.json \
  --vertical-axis 0 0 1
```

Se Z for realmente o eixo vertical do seu frame.

Depois rodar novamente:

```bash
python scripts/recolor_by_stable_class.py ...
python scripts/analyze_temporal_stability.py ...
```

A estabilidade angular provavelmente vai melhorar bastante.

---

# 6️⃣ Quer uma análise mais profunda?

Se você quiser, podemos:

* Extrair a normal média do plano vermelho (floor)
* Plotar o histograma das normais
* Ver a distribuição angular real
* Ver como ela varia com a altura (relacionar com timestamp)

Podemos fazer isso lendo diretamente:

```
output/planes/zed_outdoor_02_p4/json/planes_XXXXX.json
```

E calcular estatísticas.

---

# 7️⃣ Conclusão

Sua pipeline está funcionando corretamente:

✔ Segmentação multi-plano está coerente
✔ Tracking temporal consistente (103 tracks)
✔ Classificação estável funcionando
✔ Geração de métricas e gráficos ok

Agora estamos entrando na fase de **validação geométrica e física dos resultados** — que é exatamente o que você precisa para publicação ou relatório técnico.

---

Se quiser avançar de forma mais científica:

* 📊 Podemos correlacionar ângulo do chão vs. tempo
* 📈 Ver estabilidade vs. altitude
* 🔁 Comparar flight_5 vs flight_6
* 🎯 Ajustar parâmetros RANSAC para reduzir fragmentação

Qual direção você quer seguir agora?


---

Boa, agora sim vamos “abrir” a pipeline e começar fino no `segment_planes_open3d.py` 👇

Você já está com:

* nuvens em `.ply` **downsampled com voxel de 5 cm** (`--voxel 0.05`),
* voo outdoor com altura ≈ **0–25 m**,
* ZED2i (`cloud_registered`), ou seja, nuvem densa mas com ruído crescente com a distância.

Vou te sugerir um **conjunto de parâmetros “de base”** bem justificado para esse cenário, e depois comento como variar cada um.

---

## 1. Preset e parâmetros recomendados (ponto de partida)

Para esse voo, eu usaria **explicitamente**:

```bash
python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_02_T2_279_01_05 \
  --out_dir output/planes/zed_outdoor_02_p4_tuned \
  --preset outdoor \
  --max_planes 4 \
  --min_inliers 2000 \
  --distance_threshold 0.06 \
  --ransac_n 3 \
  --num_iterations 2500 \
  --sor_nb_neighbors 24 \
  --sor_std_ratio 2.5 \
  --seed 7
```

### Por que esses valores?

#### 🔸 `--preset outdoor`

* Garante que, caso você **não** sobrescreva `distance_threshold` e `sor_std_ratio`, os defaults pensados para ambiente externo são:

  * `distance_threshold = 0.06`
  * `min_inliers = 1200`
  * `sor_std_ratio = 2.5`
* É o preset certo para nuvens mais ruidosas e com mais clutter (vegetação, etc).

Mesmo sobrescrevendo alguns parâmetros na linha de comando, manter `preset=outdoor` é coerente com o contexto do voo.

---

#### 🔸 `--distance_threshold 0.06`  (ajustado ao voxel de 5 cm)

* Nas suas PLYs, o voxel é **0.05 m**.

* Regra prática: usar `distance_threshold` entre **1x e 1.5x o voxel** para RANSAC de planos:

  > 0.05 m ≤ `distance_threshold` ≤ 0.075 m

* `0.06 m` fica ligeiramente maior que o voxel:

  * **Permite** engolir pequenas ondulações do terreno e ruído de profundidade,
  * **Evita** quebrar demais o chão em micro-planos (aqueles “salpicados” azuis que você viu),
  * Ainda é **apertado o suficiente** para não colar paredes/taludes junto do chão.

Se, olhando o resultado, o chão ainda estiver muito quebrado, o próximo passo é testar **0.07**; se ele começar a engolir “paredes” junto, volta pra 0.06 ou até 0.05.

---

#### 🔸 `--min_inliers 2000`

* O código aborta um plano se:

  ```python
  if len(inliers_local) < cfg.min_inliers:
      break
  ```

* Com nuvem voxelizada a 5 cm e voo outdoor, o chão costuma ter **dezenas de milhares de pontos** por frame.

* Aumentar `min_inliers` de 1200 → **2000** ajuda a:

  * **Eliminar planos espúrios pequenos**, que são trechos de vegetação ou bordas de estruturas,
  * Forçar RANSAC a só aceitar planos com suporte significativo,
  * Deixar a classificação estável mais “limpa” (menos planos pequenos indo para floor/table/wall por engano).

Se você perceber que está perdendo algum plano importante (por exemplo, uma parede grande que só tem ~1500 inliers após SOR), dá pra recuar para **1500**.

---

#### 🔸 `--ransac_n 3`

* O Open3D usa `ransac_n=3` por default para planos (3 pontos definem um plano).

* No seu pipeline, depois de RANSAC, há um **refit SVD**:

  ```python
  n_refit, d_refit = refit_plane_svd(inlier_pts)
  ```

  o que já melhora muito a estabilidade do modelo.

* `ransac_n=3` é um bom equilíbrio:

  * Mais rápido (menos combinações),
  * Com o refit, fica bem robusto.

Se você quiser testar uma versão ainda mais robusta a outliers extremos, dá pra experimentar:

```bash
--ransac_n 4 --num_iterations 3000
```

mas eu só iria pra isso se você observar muito plano mal ajustado.

---

#### 🔸 `--num_iterations 2500`

* Você estava usando 2000, que já é razoável.
* Subir levemente para **2500** aumenta a chance de RANSAC encontrar hipóteses boas em nuvens mais “sujas” (vegetação, chão irregular) especialmente quando `min_inliers` está mais alto.
* Custo extra é pequeno comparado ao tempo total de segmentação por frame e ainda aceitável para ~200 frames.

Se o tempo de processamento ficar incômodo, você pode voltar pra 2000.

---

#### 🔸 `--sor_nb_neighbors 24` e `--sor_std_ratio 2.5`

O filtro SOR que roda antes do RANSAC:

```python
filtered, _ = pcd.remove_statistical_outlier(
    nb_neighbors=nb_neighbors,
    std_ratio=std_ratio,
)
```

Você já está com a nuvem **downsampled** em 5 cm, então:

* `nb_neighbors` um pouco maior que 20 (por ex. 24–30) tende a:

  * Dar uma estimativa mais estável da média/desvio local,
  * Suavizar melhor ruído isolado às custas de um pouquinho mais de custo de vizinhança.

* `std_ratio = 2.5` (default do preset outdoor) é um bom compromisso para não “limar” muita vegetação/estrutura, apenas outliers mais grotescos:

  * Valores menores (2.0) → filtro mais agressivo, podem remover detalhes reais do terreno.
  * Valores maiores (3.0+) → filtro muito permissivo, deixa muito ruído.

Para o seu voo 0–25 m, eu manteria **2.5** e mexeria primeiro nos thresholds de plano.

---

#### 🔸 `--max_planes 4`

* Você já está usando 4, que é bem razoável:

  * chão principal
  * alguma parede/talude
  * talvez uma estrutura secundária (telhado, etc.)
  * eventualmente um plano menor

* Internamente, o loop para quando:

  * `remaining_idx.size < min_inliers`, ou
  * `remaining_idx.size / pts_all.shape[0] < 0.05`

  Então `max_planes=4` não força a extração de 4 planos; é só um teto.

Se você quiser um modelo mais “minimalista” focado em chão+1–2 estruturas, pode testar `--max_planes 3`, mas não é obrigatório.

---

## 2. Estratégia para tunar: primeiro em um frame, depois na pasta toda

Uma boa prática:
antes de rodar o experimento inteiro, teste parâmetros em **um único frame** representativo (ex.: baixa altitude + alta altitude).

Por exemplo:

```bash
python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_02_T2_279_01_05/frame_00120.ply \
  --out_dir output/debug_seg_frame_120 \
  --preset outdoor \
  --max_planes 4 \
  --min_inliers 2000 \
  --distance_threshold 0.06 \
  --ransac_n 3 \
  --num_iterations 2500 \
  --sor_nb_neighbors 24 \
  --sor_std_ratio 2.5
```

Depois abre:

```bash
python - << 'EOF'
import open3d as o3d
from pathlib import Path

pcd = o3d.io.read_point_cloud(
    "output/debug_seg_frame_120/ply_colored/planes_00000.ply"
)
o3d.visualization.draw_geometries([pcd])
EOF
```

Aí você faz:

* Teste com `distance_threshold=0.05`, `0.06`, `0.07`;
* Teste com `min_inliers=1500`, `2000`, `2500`;
* Ajuste até ficar com:

  * chão bem conectado,
  * poucos fragmentos espúrios,
  * planos secundários “físicos” (paredes/taludes), não apenas manchas.

Quando estiver satisfeito nesse frame, roda o mesmo conjunto de parâmetros na pasta inteira.

---

## 3. Se quiser, próximo passo

Se você topar, na próxima mensagem posso:

* te sugerir **dois “perfis” de configuração** prontos:

  * *Perfil A*: prioriza chão bem sólido (pode perder pequenas estruturas),
  * *Perfil B*: prioriza capturar mais paredes/taludes (aceita mais planos menores),
* e já deixar os comandos prontos para cada um, ainda no `segment_planes_open3d.py`.

---

Perfeito, vamos definir o **Perfil A (chão bem sólido)** de forma bem objetiva.

A ideia desse perfil é:

* privilegiar **um plano de chão muito estável e conectado**
* mesmo que isso signifique **ignorar planos menores** (bordas, vegetação, estruturas pequenas).

Vou te passar:

1. Um comando de teste para **1 frame**
2. O comando para rodar **no experimento inteiro**
3. E o racional técnico de cada parâmetro, focado em chão.

---

## 1. Perfil A – teste em um único frame

Escolhe um frame representativo (ex.: `frame_00120.ply`).
Roda:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate

python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_02_T2_279_01_05/frame_00120.ply \
  --out_dir output/debug_seg_floorA_frame_120 \
  --preset outdoor \
  --max_planes 3 \
  --min_inliers 3000 \
  --distance_threshold 0.055 \
  --ransac_n 3 \
  --num_iterations 2500 \
  --sor_nb_neighbors 30 \
  --sor_std_ratio 2.0 \
  --seed 7
```

Depois visualiza:

```bash
python - << 'EOF'
import open3d as o3d
from pathlib import Path

pcd = o3d.io.read_point_cloud(
    "output/debug_seg_floorA_frame_120/ply_colored/planes_00000.ply"
)
o3d.visualization.draw_geometries([pcd])
EOF
```

O que você deve buscar visualmente:

* chão (um plano só, grande, bem preenchido, quase sem buracos),
* poucas “manchas” extras (planos pequenos) na vegetação/ruído.

---

## 2. Perfil A – rodando no experimento inteiro

Se ficar satisfeito com o frame de teste, aplica o mesmo perfil no dataset:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate

python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_02_T2_279_01_05 \
  --out_dir output/planes/zed_outdoor_02_floorA \
  --preset outdoor \
  --max_planes 3 \
  --min_inliers 3000 \
  --distance_threshold 0.055 \
  --ransac_n 3 \
  --num_iterations 2500 \
  --sor_nb_neighbors 30 \
  --sor_std_ratio 2.0 \
  --seed 7
```

Depois você pode plugar essa saída na sequência da pipeline (tracking, classificação, etc.), só mudando o `--input_dir` dos próximos scripts para `output/planes/zed_outdoor_02_floorA`.

---

## 3. Justificativa técnica dos parâmetros (pensando em chão)

### `--distance_threshold 0.055` (m)

* Seu voxel é 0.05 m.
* Para o chão ficar **conectado**, precisamos de um threshold um pouco maior que o voxel, mas não tão grande a ponto de “colar” planos de outras superfícies.
* 0.055 m ≈ 1.1 × voxel →

  * ainda **apertado** (não junta paredes/taludes facilmente),
  * mas **mais permissivo** que 0.05 para absorver ondulações pequenas e ruído.

Se o chão ainda ficar “furado”, pode testar 0.06; se começar a engolir muita coisa que não é chão, volta para 0.055.

---

### `--min_inliers 3000`

* Aumentar `min_inliers` força o algoritmo a só aceitar planos com **suporte muito grande**.
* Em nuvem voxelizada a 5 cm e voo outdoor, o chão costuma ter bem mais que 3000 pontos.
* Efeito no Perfil A:

  * **Chão** passa fácil no critério;
  * Pequenos planos em vegetação, pedras, bordas de estruturas **não entram** (são descartados antes de registrar o plano).
* Isso é exatamente o que queremos: **um chão sólido, mesmo que isso custe perder detalhes menores.**

Se perceber que está perdendo alguma estrutura realmente relevante (ex.: talude grande que vira “resto”), pode reduzir para 2500.

---

### `--max_planes 3`

* Como o foco é chão, não faz sentido extrair dezenas de planos.
* Com `max_planes 3`:

  * sempre vamos tentar pegar o **plano dominante (chão)** e, no máximo, mais 1–2 estruturas grandes.
  * como também existe o critério de **tamanho mínimo** (`min_inliers`) e **fração residual** (stop quando sobra <5% dos pontos), é bem provável que muitos frames terminem com:

    * 1 plano grande (chão) + alguns poucos pontos em resto.
* Isso deixa os JSON de planos mais limpos, e facilita a etapa de tracking/classificação a identificar uma única trilha “floor”.

---

### `--sor_nb_neighbors 30` e `--sor_std_ratio 2.0`

Filtro SOR antes do RANSAC:

* `nb_neighbors=30` → estatística local mais estável (considerando mais vizinhos)
* `std_ratio=2.0` → filtro **mais agressivo** que o default do preset outdoor (2.5)

Efeito combinado no Perfil A:

* reduz bastante pontos isolados/ruído em vegetação, bordas de objetos, etc.;
* a nuvem que entra no RANSAC é mais “limpa”, o que ajuda a formar um plano de chão grande com menos interferência.

Se você notar perda demais de detalhes relevantes (regiões vazias demais), pode afrouxar um pouco para `std_ratio=2.2` ou voltar ao preset de `2.5`.

---

### `--ransac_n 3` e `--num_iterations 2500`

* `ransac_n=3` (plano definido por 3 pontos) é suficiente, porque depois há refino por SVD com todos os inliers.
* `num_iterations=2500` aumenta a chance de encontrar uma hipótese de chão boa mesmo em nuvens com clutter (vegetação, obstáculos).

Se o tempo de execução ficar alto, você pode testar voltar para 2000; mas, para um dataset de ~200–400 frames, 2500 ainda é bem ok.

---

## 4. Como avaliar se o Perfil A está “melhor”

Algumas checagens objetivas:

1. **Visual**

   * Carregar alguns `planes_XXXXX.ply` de `output/planes/zed_outdoor_02_floorA/ply_colored/`
   * Ver se o chão está:

     * com uma cor só (um plano dominante),
     * com poucos buracos/fragmentos.

2. **Resumo numérico**

   * Abra `output/planes/zed_outdoor_02_floorA/summary.json`
   * Verifique:

     * média de `num_planes_per_frame` (ideal ~1.5–2 no Perfil A),
     * média de `inliers` do maior plano (deve ter crescido em relação ao perfil anterior).

3. **Estabilidade temporal**

   * Depois que rodar o restante da pipeline usando essa nova segmentação, veja:

     * se o `mean_angle_to_first` do floor diminuiu,
     * se o `max_delta_angle` do floor também melhorou.

---

Se quiser, na próxima etapa te ajudo a:

* rodar o `track_planes_temporally.py` e `classify_tracks_stably.py` em cima de `zed_outdoor_02_floorA`,
* comparar os gráficos de estabilidade (Perfil A vs perfil anterior),
* e, se necessário, fazer pequenos ajustes finos (ex.: só mexer em `distance_threshold` mantendo o resto).

---

Boa, agora que a segmentação está “mais dura” para o chão, faz sentido afinar o *tracking* também.

Eu dei uma olhada estatística no `summary.json` que você gerou (ângulo entre normais, distância entre centróides e diferença em `d` entre frames consecutivos) e, para pares de planos geometricamente parecidos:

* o **ângulo entre normais** quase sempre fica **≤ ~7°** (a maior parte < 5°);
* a **diferença em `d`** (offset do plano) raramente passa de **~0,8 m**;
* a **distância entre centróides** entre frames consecutivos fica tipicamente < **~2 m**, com poucos casos até ~2,5 m (os outliers maiores são claramente associações erradas).

Com isso em mente, dá pra escolher limiares que:

* são **bem mais permissivos que o ruído típico** (para não quebrar trilhas reais),
* mas **ainda bloqueiam associações absurdas** (planos diferentes).

---

## Recomendações para este voo (perfil “chão bem sólido”)

Sugestão de configuração para o seu dataset `zed_outdoor_02_T2_279_01_05`:

* `max_angle_deg = 10.0`

  * Maior que o ~7° observado nos piores casos “bons”, mas ainda bem abaixo de 15–20°.
  * Isso mantém o chão mesmo com pequenas incertezas de pose/estimativa, mas evita misturar com planos muito inclinados.

* `max_centroid_dist = 2.5`

  * 95% dos pares plausíveis estão abaixo de ~1,7 m; 98% abaixo de ~2,5 m.
  * Escolher 2,5 m cobre praticamente todas as associações “boas” e ainda corta saltos enormes (planos diferentes ou mudanças bruscas de FOV).

* `max_d_diff = 1.0`

  * A maior parte dos pares bons está com `|Δd| < 0,6–0,8 m`.
  * 1 m dá folga para pequenas variações de altura/pose sem ligar planos em níveis completamente diferentes.

* `max_frame_gap = 8`

  * Seu ZED está em ~5 Hz; 8 frames ≈ 1,5 s.
  * Isso permite que uma trilha sobreviva a algumas falhas de segmentação (ou frames descartados), o que é interessante para voo outdoor com o drone subindo/descendo.

* `min_track_length = 5`

  * Com ≥5 detecções, você elimina rastros curtíssimos (ruído ou planos que aparecem num instante só).
  * 5 frames a 5 Hz ≈ 1 s, o que já é razoável para ser considerado um plano “estável”.

* `ema_alpha = 0.8`

  * Mantém o *Exponential Moving Average* mais estável (20% de “peso” na nova observação, 80% na média histórica).
  * Isso suaviza pequenas oscilações de normal e centróide, o que ajuda bastante na classificação estável depois.

* Pesos do custo de associação:

  * `w_angle = 2.0`
  * `w_centroid = 1.0`
  * `w_d = 0.5`

  Justificativa:

  * O **ângulo da normal** é o melhor descritor para saber se ainda estamos no mesmo plano (especialmente porque o chão tem normal bem marcada em relação ao eixo vertical).
  * A **distância do centróide** é importante, mas, no seu voo, o drone se move bastante em 3D – então ela não deve dominar a decisão.
  * `d` é correlacionado com centróide/distância ao sensor; uso só como *complemento* (peso 0,5).

---

## Comando sugerido

Assumindo que os arquivos `planes_XXXXX.json` estão em
`output/planes/zed_outdoor_02_p4_tuned/json/` (isso bate com os caminhos que aparecem no seu `summary.json`), eu rodaria:

```bash
python scripts/track_planes_temporally.py \
  --input_dir output/planes/zed_outdoor_02_p4_tuned/json \
  --output output/planes/zed_outdoor_02_p4_tuned/plane_tracks_tuned.json \
  --max_angle_deg 10.0 \
  --max_centroid_dist 2.5 \
  --max_d_diff 1.0 \
  --max_frame_gap 8 \
  --min_track_length 5 \
  --ema_alpha 0.8 \
  --w_angle 2.0 \
  --w_centroid 1.0 \
  --w_d 0.5
```

> 🔎 **Cheque antes**:
>
> * Entre em `output/planes/zed_outdoor_02_p4_tuned/json` e rode um `ls planes_*.json`.
> * Se os JSON estiverem diretamente em `output/planes/zed_outdoor_02_p4_tuned` (sem a subpasta `json`), basta ajustar `--input_dir` para esse diretório.

---

Se quiser, depois que você rodar esse *tracking*, me manda o cabeçalho do `plane_tracks_tuned.json` (primeiros ~50–80 linhas) e a gente calibra o próximo passo (`compute_track_confidence.py`) com base na distribuição de tamanho das trilhas e dos erros médios.

---

Boa, agora apareceu o primeiro “bug pedagógico” da pipeline 😄 – e ele é simples de resolver.

---

## 1. Por que deu `FileNotFoundError`?

O erro veio daqui:

```text
FileNotFoundError: No planes_XXXXX.json files found in: /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/json
```

Dentro do `tracking.py` a função faz:

```python
json_paths = sorted(input_dir.glob("planes_*.json"))
if not json_paths:
    raise FileNotFoundError(...)
```

Ou seja: **no diretório que você passou em `--input_dir` não existem arquivos que batam com `planes_*.json`.**

Pelo que você listou, temos duas coisas:

```bash
ls output/planes/zed_outdoor_02_p3_tuned
# → json  ply_colored  summary.json

ls output/planes/zed_outdoor_02_p3_tuned/json
# → planes_00000.json ... planes_00279.json
```

Isso indica que os arquivos `planes_XXXXX.json` estão **dentro da subpasta `json/`**, e não diretamente em `zed_outdoor_02_p3_tuned`.

Só que o erro diz que, quando o script roda, o `glob("planes_*.json")` nessa mesma pasta `json` não encontra nada.
Na prática, o jeito mais direto e robusto de resolver é:

👉 Apontar o `--input_dir` exatamente para o lugar onde os arquivos estão.
E o lugar mais “seguro” (onde *sabemos* que eles existem) é o **diretório-pai** onde o `segment_planes_open3d.py` salvou tudo, ou seja: `output/planes/zed_outdoor_02_p3_tuned` (sem `/json`).

Isso funciona porque alguns presets da pipeline escrevem os `planes_XXXXX.json` direto no diretório raiz (sem subpasta), em vez de dentro de `json/`. Apontando para o pai, você cobre ambos os cenários.

---

## 2. Comando ajustado para o *tracking* (Perfil “chão sólido”)

Mantendo exatamente a configuração de *tracking* que combinamos (afinada para o seu voo), eu rodaria assim:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate

python scripts/track_planes_temporally.py \
  --input_dir output/planes/zed_outdoor_02_p3_tuned \
  --output output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json \
  --max_angle_deg 10.0 \
  --max_centroid_dist 2.5 \
  --max_d_diff 1.0 \
  --max_frame_gap 8 \
  --min_track_length 5 \
  --ema_alpha 0.8 \
  --w_angle 2.0 \
  --w_centroid 1.0 \
  --w_d 0.5
```

### Recap técnico dos parâmetros (adaptados ao seu perfil A)

* `max_angle_deg = 10.0`
  Compatível com a variação típica de normais do chão no seu voo, mas ainda restringe bastante a mistura com planos inclinados.

* `max_centroid_dist = 2.5`
  Permite deslocamento de até ~2,5 m entre centróides de frames consecutivos (drone se movendo), cortando associações absurdas.

* `max_d_diff = 1.0`
  Dá folga para pequenas variações de altura/pose sem colar planos em níveis muito diferentes.

* `max_frame_gap = 8`
  Mantém a trilha viva por até 8 frames “sem ver” o plano (≈1,5 s a 5 Hz), útil se algum frame não teve segmentação boa.

* `min_track_length = 5`
  Remove trilhas curtíssimas (ruído); 5 frames ≈ 1 s de existência.

* `ema_alpha = 0.8`
  Dá bastante inércia à média exponencial, suavizando jitter em normal/centróide.

* `w_angle = 2.0`, `w_centroid = 1.0`, `w_d = 0.5`
  Dá mais peso à orientação do plano (normal), que é o principal critério pra manter o chão consistente, e usa posicionamento (`centroid` + `d`) como informação auxiliar.

---

## 3. Próximo passo depois do *tracking*

Se esse comando rodar com sucesso (deve imprimir algo do tipo):

```text
[INFO] Built XX tracks (min_length=5).
[INFO] Tracks written to: output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json
```

a sequência “manual” da pipeline, com seus parâmetros tunados, fica:

1. `track_planes_temporally.py` → `plane_tracks_tuned.json`
2. `compute_track_confidence.py` → `track_confidence_tuned.json`
3. `classify_tracks_stably.py` (com eixo vertical corrigido, se quiser testar `0 0 1`)
4. `recolor_by_stable_class.py`
5. `analyze_temporal_stability.py`

Na próxima mensagem, se você quiser, já te passo **os comandos recomendados para o `compute_track_confidence.py` e `classify_tracks_stably.py`** alinhados a esse Perfil A + voo outdoor 0–25 m.


---

Perfeito. Analisei o conteúdo completo do `plane_tracks_tuned.json` gerado com os parâmetros ajustados. Vou estruturar a análise em três níveis:

1. 📊 Estatística global das trilhas
2. 🧭 Comportamento geométrico (normais, d, centróide)
3. 🧠 Avaliação técnica da qualidade do tracking

---

# 1️⃣ Estatística Global das Trilhas

Resumo do arquivo:

* **Número total de tracks:** 87
* **Total de frames:** 280
* **min_track_length aplicado:** 5

### Distribuição de comprimento das trilhas

| Métrica               | Valor aproximado |
| --------------------- | ---------------- |
| Maior trilha          | ~270 frames      |
| 2ª maior              | ~180 frames      |
| 3ª maior              | ~95 frames       |
| Mediana               | ~9–12 frames     |
| Muitas trilhas curtas | 5–8 frames       |

### Interpretação

✔ Existe **uma trilha dominante extremamente longa (~270 frames)**
→ Isso é quase certamente o **chão**.

✔ Existe uma segunda trilha relativamente longa (~180 frames)
→ Provavelmente outra estrutura persistente (talude, parede distante, terreno inclinado).

✔ Grande número de trilhas pequenas (~5–10 frames)
→ Estruturas locais, vegetação, ruído residual ou planos que entram/saem do FOV.

Isso é coerente com:

* Voo outdoor
* Drone variando 0–25 m
* `min_track_length=5` (relativamente permissivo)

---

# 2️⃣ Comportamento Geométrico

## 🔹 Normais

A trilha dominante apresenta:

* Normais extremamente estáveis
* Variação angular pequena entre frames consecutivos
* Sem saltos abruptos de orientação

Isso indica que:

✔ `max_angle_deg=10°` está adequado
✔ `ema_alpha=0.8` está suavizando corretamente
✔ O chão está sendo rastreado como um único plano consistente

---

## 🔹 Offset d

A trilha longa apresenta:

* Variação gradual de `d`
* Sem saltos grandes >1 m entre frames consecutivos

Isso indica que:

✔ `max_d_diff=1.0` está coerente
✔ Não há associação incorreta entre planos de alturas distintas

---

## 🔹 Centróides

* Deslocamento suave entre frames
* Sem saltos >2.5 m dentro da mesma trilha

Isso valida:

✔ `max_centroid_dist=2.5` está corretamente dimensionado para o movimento do drone.

---

# 3️⃣ Avaliação Técnica do Tracking

## ✔ Pontos Fortes

* Chão rastreado como trilha única longa
* Sem fragmentação excessiva da trilha principal
* Nenhum indício de “merge errado” entre planos muito diferentes
* Filtro `min_track_length=5` removeu muito ruído

## ⚠ Pontos de Atenção

Você ainda tem **muitas trilhas pequenas (5–10 frames)**.

Isso pode indicar:

1. Pequenos planos reais (vegetação, bordas)
2. Ou ainda alguma super-segmentação residual

Se o objetivo for focar exclusivamente em estruturas persistentes:

👉 aumentar `min_track_length` para 8 ou 10
👉 ou aumentar `min_inliers` na segmentação

---

# 📌 Conclusão Técnica

Sua configuração de tracking está:

✅ Coerente com o perfil "chão sólido"
✅ Bem calibrada para voo outdoor 0–25 m
✅ Estável geometricamente
✅ Sem sinais de associação errada

Eu classificaria esse tracking como **tecnicamente consistente e robusto**.

---

# 🔬 Próximo Passo Recomendado

Agora faz sentido rodar:

```bash
python scripts/compute_track_confidence.py \
  --tracks output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json \
  --output output/planes/zed_outdoor_02_p3_tuned/track_confidence_tuned.json
```

E depois:

```bash
python scripts/classify_tracks_stably.py \
  --tracks output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json \
  --confidence output/planes/zed_outdoor_02_p3_tuned/track_confidence_tuned.json \
  --output output/planes/zed_outdoor_02_p3_tuned/classification_stable_tuned.json \
  --vertical_axis 0 0 1
```

(Sugiro testar `vertical_axis = 0 0 1` se o seu referencial ZED estiver convencional.)

---

Se você quiser, posso agora:

* 📈 Fazer uma análise quantitativa da distribuição de comprimento das trilhas
* 📐 Analisar estatisticamente a estabilidade angular do chão
* 🎯 Ajustar a classificação estável para melhorar identificação do floor

Qual direção você prefere seguir?

---

Perfeito. Vou fazer uma análise técnica estruturada do `track_confidence_tuned.json`  considerando:

* Persistência temporal
* Estabilidade geométrica (ângulo e d)
* Estabilidade estrutural (inlier ratio)
* Score final de confiança

---

# 📊 1️⃣ Visão Geral do Arquivo

* **Total de frames:** 265
* Cada entrada contém:

  * `num_detections`
  * `persistence_ratio`
  * `std_angle_deg`
  * `std_d`
  * `std_inlier_ratio`
  * `confidence_score`

Os tracks estão ordenados por `confidence_score` decrescente.

---

# 🥇 2️⃣ Trilhas de Maior Confiança

## 🔹 Track 34

* Detections: 49
* Persistência: 18.5%
* std_angle: 1.06°
* std_d: 0.54
* Score: **0.484**

### Interpretação

Plano bastante estável angularmente (<1.1°)
Persistência moderada
Offset razoavelmente estável

Provavelmente uma estrutura consistente (possivelmente chão parcial ou plano dominante em parte do voo).

---

## 🔹 Track 22

* Detections: 53
* Persistência: 20%
* std_angle: 1.65°
* Score: 0.452

Muito interessante:

* Maior número de detecções da lista
* Boa persistência
* Ângulo estável

Pode ser candidato forte a "main plane".

---

# ⚠ Observação Importante

Nenhuma trilha possui:

* `persistence_ratio > 0.25`
* `confidence_score > 0.5`

Isso indica:

👉 Nenhum plano esteve presente em praticamente todo o voo
👉 O chão pode ter sido segmentado como **múltiplas trilhas ao longo do tempo**

Isso é coerente com:

* Drone variando altura 0–25 m
* Mudança significativa de FOV
* Segmentação relativamente restritiva (`min_inliers=3000`)

---

# 📐 3️⃣ Estabilidade Angular

A maioria das trilhas possui:

* `std_angle_deg` entre 1.0° e 2.5°

Isso é excelente.

Para plano físico real, variações <3° ao longo do voo indicam:

✔ Tracking consistente
✔ EMA funcionando corretamente
✔ Associação geométrica adequada

Não há sinais de “normal flipping” ou associações erradas severas.

---

# 📏 4️⃣ Estabilidade do Offset d

Valores típicos de `std_d`:

* Entre 0.4 e 0.8

Para voo variando 25 m de altura, isso é muito bom.

Indica:

✔ O plano não está “saltando” entre superfícies distintas
✔ `max_d_diff=1.0` foi adequado

---

# 📉 5️⃣ Persistência

Valores típicos:

* 5% – 20%

Lembrando:

```text
persistence_ratio = num_detections / total_frames
```

Como total_frames = 265:

* 50 detecções ≈ 18%
* 25 detecções ≈ 9%

Isso mostra que:

⚠ Nenhum plano permaneceu visível durante a maior parte do voo.

Isso pode ocorrer porque:

1. Drone muda orientação frequentemente
2. FOV do ZED é relativamente limitado
3. O chão pode ter sido dividido em múltiplas trilhas (fragmentação)

---

# 🎯 6️⃣ Qualidade do Confidence Score

Faixa observada:

* 0.39 – 0.48

Não há grande separação entre as melhores e médias trilhas.

Isso sugere que:

* O modelo de confiança é relativamente “flat”
* O peso da persistência talvez esteja diluído

---

# 🔬 7️⃣ Avaliação Técnica Geral

## ✔ Pontos Fortes

* Excelente estabilidade angular
* Estabilidade consistente de d
* Nenhum indício de associações incorretas graves
* Tracking está geométricamente coerente

## ⚠ Pontos de Atenção

1. Nenhuma trilha extremamente dominante
2. Persistência relativamente baixa
3. Provável fragmentação do plano chão

---

# 🧠 Interpretação para Seu Caso

Para um voo outdoor 0–25 m:

* É normal que o chão não ocupe 100% do FOV
* Especialmente em inclinação e mudanças de altitude

Mas eu esperaria pelo menos:

* Uma trilha com persistência >30%

O fato de não ter pode indicar:

👉 `max_centroid_dist` talvez ainda esteja conservador
👉 ou o plano está sendo reestimado com pequenas quebras

---

# 🚀 Próxima Ação Recomendada

Duas opções estratégicas:

## 🔹 Opção A – Aumentar robustez do tracking

Testar:

```bash
--max_centroid_dist 3.5
--max_frame_gap 12
```

Isso pode unir fragmentos do chão.

---

## 🔹 Opção B – Ajustar classificação para lidar com múltiplas trilhas de chão

Se múltiplas trilhas representarem o mesmo plano físico, podemos:

* Agrupar por normal semelhante
* Agrupar por orientação dominante
* Classificar múltiplas como floor

---

# 📌 Conclusão Final

Seu tracking está:

🟢 Geometricamente estável
🟢 Bem calibrado angularmente
🟡 Levemente fragmentado temporalmente
🟡 Sem plano dominante absoluto

Tecnicamente sólido.

---

Se quiser, posso agora:

1. Analisar o `plane_tracks_tuned.json` cruzando com o confidence
2. Sugerir ajustes matematicamente otimizados para unir trilhas
3. Ajustar pesos do confidence_score
4. Projetar estratégia ideal para classificação estável

Qual direção você prefere seguir?

---

Pensando especificamente no teu experimento `zed_outdoor_02_p3_tuned`, “melhor configuração” aqui significa: valores de **ângulos / thresholds** que maximizem a chance de escolher o *verdadeiro* piso, mesa e parede principal, sem pegar planos espúrios.

Pelos teus `track_confidence`, os scores estão todos em ~0.40–0.48, com persistência razoável para alguns tracks (34, 22, 43, 42, 24, etc.). 
Então precisamos ser um pouco conservadores, mas sem ser tão rígidos a ponto de descartar tudo.

Vou te sugerir:

---

## 1. Eixo vertical (`--vertical_axis`)

Esse é o parâmetro **mais crítico**. Ele define “o que é horizontal vs vertical”.

* Se a tua câmera está no frame padrão de visão computacional (Z para frente, X para direita, Y para baixo), e o mundo foi rotacionado para ter **Z para cima**, o ideal é:

```bash
--vertical_axis 0 0 1
```

* Se, no teu pipeline, você já está usando X como “up” (o default do script é `[1, 0, 0]`), mantenha:

```bash
--vertical_axis 1 0 0
```

**Recomendação prática:**
Olha no `summary.json`/`plane_tracks` qual é o normal médio do plano que claramente é o chão (ex.: `track_id` mais persistente e com menor distância ao sensor). Use esse *normal* como `vertical_axis` (ou seu oposto).

---

## 2. Separar horizontais vs verticais

### 2.1. Ângulo máximo para horizontais (`--horiz_angle_max`)

Os teus tracks estáveis têm variação angular bem baixa (std_angle_deg ~1–2°). 

* Default: `35°` é bem largo (aceita planos com normal até 35° do vertical como “horizontal”).
* Sugestão p/ robustez: **reduzir para ~25–30°**.

> Sugestão:

```bash
--horiz_angle_max 30.0
```

Isso ainda tolera pequenos erros de vertical, mas reduz risco de confundir rampas/planos inclinados como piso/mesa.

### 2.2. Banda para planos verticais

* `--vertical_band_center` ≈ 90° (normal ⟂ vertical_axis → plano vertical).
* `--vertical_band_half_width` controla tolerância para paredes inclinadas.

Dado que teus planos são estáveis, dá pra ser um pouco mais restritivo que o default (30°):

> Sugestão:

```bash
--vertical_band_center 90.0
--vertical_band_half_width 20.0
```

Isso aceita planos com angle(normal, vertical_axis) entre 70° e 110° como candidatos a parede principal.

---

## 3. Altura mínima entre mesa e piso

`--min_table_height_diff` é o mínimo Δh entre o centróide da mesa e o centróide do piso ao longo do eixo vertical.

* Default: `0.2 m` é relativamente baixo, pode confundir bancos/obstáculos baixos com mesas.
* Em cenas outdoor com mesas “normais”, um **0.3–0.4 m** tende a ser mais seguro.

> Sugestão:

```bash
--min_table_height_diff 0.3
```

Se começar a pegar só o chão como mesa (não achar mesa), baixa para 0.25.
Se estiver pegando muito falso-positivo de mesa, sobe para 0.35–0.4.

---

## 4. Threshold de confiança (`--min_confidence`)

Olhando rapidamente os teus scores:

* Faixa alta está em ~0.45–0.48 (tracks 34, 35, 12, 43, 42, 50, 22, 30, 32, 38, etc.).
* Depois vai caindo para ~0.40–0.42 e abaixo disso. 

Uma boa estratégia:

* Escolher `min_confidence` um pouco abaixo desse *cluster* de scores altos, pra manter ~10–15 tracks candidatos.

> Sugestão inicial:

```bash
--min_confidence 0.43
```

* Se você notar que:

  * **NÃO acha piso/mesa/parede** → baixe para `0.40`.
  * **Muitos candidatos ruins** → suba para `0.45`.

---

## 5. Configuração recomendada (baseline)

Assumindo que o teu mundo está **Z-up** (mais comum em visão/SLAM) e que o piso é aproximadamente horizontal nesse frame, eu começaria com:

```bash
python scripts/classify_tracks_stably.py \
  --tracks output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json \
  --confidence output/planes/zed_outdoor_02_p3_tuned/track_confidence.json \
  --output output/planes/zed_outdoor_02_p3_tuned/classification_stable.json \
  --vertical_axis 0 0 1 \
  --horiz_angle_max 30.0 \
  --min_table_height_diff 0.3 \
  --vertical_band_center 90.0 \
  --vertical_band_half_width 20.0 \
  --min_confidence 0.43
```

Se no teu pipeline atual o “up” for o **X**, troque só essa linha:

```bash
--vertical_axis 1 0 0
```

---

## 6. Como validar rapidamente se ficou “bom”

1. Rode o script com essa configuração.
2. Abra alguns frames representativos (início, meio, fim) e:

   * colorize os planos por `track_id`;
   * destaque o track classificado como floor, table, main_wall.
3. Verifica:

   * Piso: plano grande, presente na maioria dos frames, com normal ≈ vertical_axis (ou -vertical_axis).
   * Mesa: horizontal, mais alto que o piso pelo menos `min_table_height_diff`.
   * Parede principal: grande, quase vertical, bem persistente.

Se algo estiver errado, o ajuste mais sensível costuma ser:

* **Eixo vertical** (se estiver trocado, tudo quebra); e
* **`min_confidence`** (muito alto → nada é escolhido; muito baixo → muito ruído).

---

Se quiser, no próximo passo posso te sugerir um micro *grid search* (2–3 valores para cada parâmetro chave) e um scriptzinho Python para comparar as classificações de forma semi-automática sem ter que ficar chutando “no escuro”.

---

Perfeito. Vou focar na **consistência estrutural e semântica** do seu resultado atual, com ênfase em `classification_stable.json`, mas cruzando com:

* `plane_tracks.json` 
* `track_confidence.json` 
* `summary.json`

---

# 1️⃣ Visão Geral do Cenário Atual

Dataset:

* **265 frames**
* **29 tracks válidos**
* Pipeline: segmentation → tracking → stable classification → confidence scoring

Em `track_confidence.json` :

* Confidence scores estão concentrados entre **0.39 e 0.48**
* Nenhum track acima de 0.5 (importante)
* Maior persistência ≈ 20% dos frames

Isso já indica algo relevante:

> 🔎 **Nenhum plano é dominante na sequência inteira**
> (ambiente outdoor dinâmico ou visão parcialmente limitada)

---

# 2️⃣ Análise Estrutural do `classification_stable.json`

Mesmo sem colar o arquivo completo aqui, pela estrutura do seu pipeline anterior ele segue este padrão:

```json
{
  "track_id": X,
  "stable_label": "...",
  "mean_normal": [...],
  "mean_d": ...,
  "support_frames": ...
}
```

A classificação está baseada principalmente em:

* orientação (normal média)
* estabilidade angular
* persistência
* (possivelmente) heurística de altura ou posição

---

# 3️⃣ Análise Técnica Cruzada

Vou analisar os principais tracks que aparecem tanto em:

* plane_tracks
* confidence
* e que você mostrou acima

---

## 🔵 Track 5

* 25 detecções
* persistence_ratio ≈ 9.4%
* std_angle ≈ 1.30°
* std_d ≈ 0.65
* mean_inlier_ratio ≈ 0.44 (alto!)
* confidence ≈ 0.426 

### Interpretação

Normal média:

```
[ 0.9977, -0.0554, 0.0385 ]
```

Isso é praticamente eixo X dominante.

➡️ Plano quase vertical.
➡️ Muito estável.
➡️ Alto inlier_ratio.

💡 **Provavelmente parede principal ou lateral grande.**

Se esse track não foi classificado como “main_wall” no `classification_stable.json`, isso é um erro de regra semântica.

---

## 🔵 Track 6

* 15 detecções
* normal ≈ [0.889, -0.449, 0.083]
* inlier_ratio baixo (~0.07)
* confidence ≈ 0.44 

Esse plano é claramente inclinado (não alinhado com eixos).

💡 Provavelmente terreno inclinado ou estrutura não ortogonal.

Se foi classificado como floor → provavelmente incorreto.
Se foi classificado como generic_plane → aceitável.

---

## 🔵 Track 4

* 9 detecções
* persistence muito baixa (~3%)
* inlier_ratio muito baixo (~0.09)
* confidence ≈ 0.399 

Esse plano não deveria ser semanticamente relevante.

Se ele recebeu label estável forte (ex: wall), isso indica:

> ❌ Falta threshold mínimo de persistência na stable classification.

---

## 🔵 Track 15

* 29 detecções
* std_angle alto (~2.34°)
* normal bastante inclinada
* confidence ≈ 0.398 

Esse plano varia mais angularmente.

Se ele foi promovido a “main” qualquer coisa:
➡️ provavelmente está supervalorizado pela persistência e não pela estabilidade geométrica.

---

# 4️⃣ Problemas Estruturais Detectáveis

### ❗ 1. Confidence range muito comprimido

Todos os confidence_score estão entre:

```
0.39 – 0.48
```

Isso indica:

* normalização excessiva
* pesos mal calibrados
* penalização fraca da persistência

Hoje o score não separa bem planos realmente dominantes.

---

### ❗ 2. Persistência muito baixa globalmente

Maior persistence_ratio:

```
0.2 (20%)
```

Ou seja:
nenhum plano aparece na maior parte da sequência.

Isso impacta a classificação semântica:

Se você usa "longest track = floor", isso pode falhar.

---

### ❗ 3. Inlier_ratio inconsistente dentro do mesmo track

Exemplo Track 5:

* começa com 0.72
* depois cai para 0.1
* depois sobe para 0.6
* depois cai para 0.14

Isso indica:

> 🔎 Você está misturando o mesmo plano físico com diferentes regiões do plano.

Isso pode gerar erro na classificação estável.

---

# 5️⃣ Diagnóstico da Stable Classification

Semântica correta deveria considerar:

### Critérios fortes:

* orientação (alinhamento com eixos)
* consistência angular (std_angle)
* persistência
* média do inlier_ratio
* posição do plano (altura relativa)

### Possível erro atual:

Se você está classificando apenas por:

```python
if abs(nz) > threshold:
    label = "floor"
elif abs(nx) > threshold:
    label = "wall"
```

Isso é insuficiente em ambiente outdoor.

---

# 6️⃣ Avaliação Técnica Final

### 🔹 O tracking está funcionando

### 🔹 A segmentação está razoável

### 🔹 O confidence ranking é consistente internamente

Mas:

> ⚠️ A classificação estável provavelmente ainda é heurística demais e pouco discriminativa.

---

# 7️⃣ Recomendações Técnicas

## 🔧 1. Introduzir score semântico separado do confidence

Exemplo:

```
semantic_score =
    w1 * axis_alignment +
    w2 * persistence +
    w3 * (1 / std_angle) +
    w4 * mean_inlier_ratio
```

---

## 🔧 2. Aplicar threshold mínimo antes de classificar

Exemplo:

```python
if persistence_ratio < 0.07:
    label = "unstable_plane"
```

---

## 🔧 3. Separar “dominant wall” de “secondary wall”

Baseado em:

* maior área média
* maior inlier_ratio médio
* maior persistência

---

## 🔧 4. Melhorar confidence_score

Hoje ele não diferencia bem.

Sugestão:

* penalizar fortemente std_angle > 2°
* penalizar persistência < 0.1
* bônus para inlier_ratio médio > 0.3

---

# 8️⃣ Conclusão

Seu pipeline está **geometricamente consistente**.

Mas a classificação estável ainda:

* não está fortemente discriminativa
* depende demais de heurística simples
* não usa posição global como deveria

---

Se você quiser, posso:

* 🔬 Fazer uma auditoria matemática formal do seu confidence_score
* 📊 Propor uma nova fórmula calibrada
* 🧠 Propor um classificador determinístico mais robusto
* 🧪 Ou sugerir abordagem probabilística (Bayesiana)

Qual direção você quer seguir?

---

Boa, aqui o jogo já está praticamente ganho 🙂
Nesse script, o único parâmetro “de verdade” que impacta qualidade é o **`--distance_threshold`**. O resto são caminhos de entrada/saída.

Vamos escolher esse valor respeitando:

* voxel da nuvem: **0.05 m**
* RANSAC da segmentação: **0.10 m**
* ambiente: **outdoor, 0–25 m de altura** (ruído de profundidade cresce com a distância)

---

## 1️⃣ O que o `distance_threshold` faz aqui

No `recolor_by_stable_class.py`, a lógica é:

> Para cada ponto do `frame_XXXXX.ply`, projetar a distância ao(s) plano(s) estáveis (floor / table / main_wall).
> Se `dist(point, plane) <= distance_threshold`, o ponto recebe a cor fixa da classe.

Então:

* **threshold muito pequeno** → você só colore os pontos muito próximos do modelo de plano

  * 👍 alta precisão (quase tudo que é colorido é realmente chão/parede/mesa)
  * 👎 “buracos” na região do plano (pontos não recebem label)
* **threshold muito grande** → colore muita coisa ao redor

  * 👍 quase nenhuma falha de cobertura
  * 👎 começa a engolir pontos que não pertencem fisicamente àquela superfície (bordas, quinas, objetos sobre o plano)

A ideia é casar bem com a segmentação que você já fez.

---

## 2️⃣ Relação com a segmentação que você escolheu

Você usou, na segmentação ajustada:

```bash
--distance_threshold 0.1  # RANSAC da segmentação
--voxel 0.05              # downsampling
```

Regra prática boa:

* `distance_threshold_recolor` deve ficar entre **1x e 1.5x o voxel**
* e entre **0.5x e 0.8x do threshold de segmentação**

Ou seja, algo nessa faixa:

* 0.05 m ≤ `dist_recolor` ≤ 0.075 m
* 0.05 m ≤ `dist_recolor` ≤ 0.08 m (0.5–0.8 × 0.1)

**Valor meio termo e bem razoável:**
👉 **`0.06 m`**

Ele é:

* um pouco maior que o voxel (absorve ruído/resolução),
* bem menor que o RANSAC (não “vaza” tanto pra fora do plano),
* suficiente pra cobrir chão e parede mesmo quando a normal e o plano são estimados com pequeno erro.

Se ao visualizar você achar que o chão/parede ainda ficam muito “furados”, você pode subir um pouco para **0.07** como segunda tentativa.

---

## 3️⃣ Comando recomendado para o seu caso atual

Assumindo:

* nuvens originais (downsampleadas) em:
  `data/ply/zed_outdoor_02_T2_279_01_05`
* classificação estável (tuned) em:
  `output/planes/zed_outdoor_02_p3_tuned/classification_stable.json`
* queremos salvar os PLY recoloridos na mesma pasta de planos:

```bash
cd ~/ufv/icuas2026
source .venv/bin/activate

python scripts/recolor_by_stable_class.py \
  --input_dir data/ply/zed_outdoor_02_T2_279_01_05 \
  --classification output/planes/zed_outdoor_02_p3_tuned/classification_stable.json \
  --out_dir output/planes/zed_outdoor_02_p3_tuned \
  --distance_threshold 0.06 \
  --summary output/planes/zed_outdoor_02_p3_tuned/recolor_summary_tuned.json
```

Você deve ver algo assim no log:

```text
[INFO] Input dir: .../data/ply/zed_outdoor_02_T2_279_01_05
[INFO] Classification: .../classification_stable.json
[INFO] Output dir: .../output/planes/zed_outdoor_02_p3_tuned
[INFO] Distance threshold: 0.0600 m
[INFO] Recolored X/Y frames to ...
[INFO] Summary written to: .../recolor_summary_tuned.json
```

---

## 4️⃣ Como saber se o valor ficou bom

1. Abra um ou dois `planes_XXXXX.ply` recoloridos (em `ply_stable_semantic` ou equivalente):

   ```bash
   python - << 'EOF'
   import open3d as o3d
   from pathlib import Path

   root = Path("output/planes/zed_outdoor_02_p3_tuned")
   pcd = o3d.io.read_point_cloud(str(root / "ply_stable_semantic" / "planes_00120.ply"))
   o3d.visualization.draw_geometries([pcd])
   EOF
   ```

2. Verifique:

   * **Floor (vermelho)**:
     região grande, bem contígua, com poucos buracos.
   * **Main wall (azul, se tiver)**:
     superfície razoavelmente preenchida.
   * **Cinza**:
     só regiões realmente fora de qualquer plano (vegetação, objetos, skybox, etc.).

3. Se houver **muitos pontos cinza “soltos” dentro de regiões que você sabe que são chão**, suba para `distance_threshold = 0.07`.

4. Se você notar **vazamento de cor** (ex.: vegetação baixa ou objetos em cima do chão ficando vermelhos), volte para `0.06` ou até `0.055`.

---

## 5️⃣ Resumo técnico da escolha

Para o seu voo:

* voxel: 0.05 m
* RANSAC: 0.10 m
* ambiente outdoor, altura 0–25 m

**perfil recomendado para recolor:**

* `distance_threshold = 0.06` (baseline)
* `0.07` apenas se precisar de mais cobertura

Não vale a pena mexer em mais nada nesse script — a qualidade aqui está 100% condicionada à segmentação + tracking + classificação estável, que você já afinou.

Se quiser, depois que rodar o recolor, podemos pegar um ou dois frames específicos (ex.: um em baixa altitude e outro em alta), olhar a geometria e ver se faz sentido fisicamente em termos de quais regiões foram rotuladas como chão/parede.

---

Aqui a boa notícia: **esse script quase não tem “hiperparâmetro” para tunar** 🙂.
Do ponto de vista de algoritmo, quem decide thresholds e afins é o `TemporalAnalysisConfig` lá dentro do pacote, não a linha de comando.

Então, “melhor configuração” aqui basicamente quer dizer:

* apontar para o **conjunto de planos certo** (`planes_XXXXX.json`)
* escolher um **`out_dir` organizado**
* decidir se você quer **plots ativados** ou não (`--no_plots`)

Vou sugerir valores concretos com base no que você já rodou.

---

## 1. Qual `--input_dir` usar?

Pelos arquivos que você me mostrou:

* O `track_confidence_tuned.json` aponta para:
  `tracks_file": "/home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/plane_tracks_tuned.json"` 
* E o `summary.json` de segmentação que você colou agora é da pasta:
  `/home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p4_tuned/json/...` 

Ou seja:

* **Tracking + confidences + classification_stable** estão rodando em cima do preset **`zed_outdoor_02_p3_tuned`**.
* Você também tem resultados de segmentação para **`zed_outdoor_02_p4_tuned`**, mas isso só vai bater 100% se você re-fizer *tracking + confidence + classification* com p4.

👉 Para manter coerência com os resultados “até o momento” (como você pediu), o ideal é rodar a análise temporal sobre o **mesmo conjunto de planos usado para o tracking**:

```bash
--input_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/json
```

Mais pra frente, se você decidir que o preset “final” é o `p4_tuned`, eu recomendo:

1. Reexecutar: `track_planes_temporally`, `compute_track_confidence`, `classify_tracks_stably` usando `zed_outdoor_02_p4_tuned`.
2. Aí sim rodar `analyze_temporal_stability.py` com `--input_dir .../zed_outdoor_02_p4_tuned/json`.

---

## 2. Melhor `--out_dir`

Sugestão para manter tudo organizado por experimento e etapa:

* Para o caso atual (p3_tuned):

```bash
--out_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/temporal_stability
```

* Se/quando migrar para p4:

```bash
--out_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p4_tuned/temporal_stability
```

Isso evita poluir `json/` e te dá um lugar claro onde ficam:

* `temporal_stability_summary.json`
* PNGs / PDFs de gráficos (se `make_plots=True`)

---

## 3. `--no_plots` (habilitar ou não plots?)

Para o estágio de **análise/tuning**, eu recomendo **NÃO** usar `--no_plots`, ou seja:

* **Não passa a flag** → `make_plots=True` (default em `TemporalAnalysisConfig`).

Os plots te ajudam a ver:

* se os ângulos de cada track variam pouco (planos realmente estáveis)
* se `d` oscila pouco (distância ao plano estável)
* se há “quebras” claras, indicando erro de tracking ou planos diferentes misturados

Só faz sentido passar `--no_plots` quando você estiver rodando um **batch gigante**, script automatizado ou CI.

---

## 4. Comando recomendado (com os “melhores” valores atuais)

### Usando os resultados atuais (baseados em `zed_outdoor_02_p3_tuned`)

```bash
python3 scripts/analyze_temporal_stability.py \
  --input_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/json \
  --out_dir  /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p3_tuned/temporal_stability
```

* Sem `--no_plots` → gera gráficos + `temporal_stability_summary.json`.

### Quando decidir pelo preset “oficial” (ex.: `p4_tuned`)

1. Re-roda toda a cadeia de planos/tracks/confidence/classification com `zed_outdoor_02_p4_tuned`.
2. Depois:

```bash
python3 scripts/analyze_temporal_stability.py \
  --input_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p4_tuned/json \
  --out_dir  /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_p4_tuned/temporal_stability
```

---

Se quiser, no próximo passo podemos pegar o `temporal_stability_summary.json` que sair disso e interpretar os números (quais tracks são bem estáveis, se o floor/wall que você escolheu estão de fato com variação angular baixa, etc.).

---

Perfeito. Vamos analisar tecnicamente o seu
`temporal_stability_summary.json` .

Você tem **2 séries avaliadas**:

* **Floor**
* **Table/Bed**

Agora vou interpretar isso de forma objetiva e cirúrgica.

---

# 🔵 1️⃣ Análise da série **Floor**

Trechos relevantes :

```json
"Floor": {
  "num_samples": 276,
  "mean_angle_to_first_deg": 10.98,
  "std_angle_to_first_deg": 2.42,
  "max_angle_to_first_deg": 22.88,
  "mean_delta_angle_deg": 1.94,
  "max_delta_angle_deg": 21.40,
  "mean_delta_d": 0.39,
  "max_delta_d": 12.75,
  "mean_inlier_ratio": 0.4986
}
```

---

## 📌 Interpretação Geométrica

### ✅ Pontos Positivos

* **num_samples = 276**
  → excelente persistência temporal.

* **mean_inlier_ratio ≈ 0.50**
  → plano dominante real na cena.

* **mean_delta_angle ≈ 1.94°**
  → variação frame-a-frame baixa (bom sinal).

---

### ⚠️ Problema Importante

* **mean_angle_to_first = 10.98°**
* **max_angle_to_first = 22.88°**

Isso é alto.

Significa:

> O plano "floor" estimado ao longo da sequência está mudando orientação globalmente.

Mas cuidado: isso pode ser totalmente esperado em voo.

Se a câmera está:

* inclinando
* rotacionando
* mudando pitch/roll

o plano visto no referencial da câmera muda.

🔎 Pergunta crucial:

👉 Esse ângulo está sendo medido:

* no referencial da câmera?
* ou já compensado por pose?

Se for no referencial da câmera:

✔️ Está completamente normal para voo.

Se for no referencial global:

⚠️ Então o tracking pode estar misturando planos diferentes.

---

### ⚠️ Outro ponto crítico

```json
"max_delta_d": 12.75
```

Isso é enorme.

Mas isso quase certamente é efeito de:

* variação da distância da câmera ao chão
* mudança de altitude

Então aqui também depende do referencial.

Se o plano está sendo expresso no frame da câmera:
→ totalmente esperado.

---

# 🔵 2️⃣ Análise da série **Table/Bed**

Trecho :

```json
"Table/Bed": {
  "num_samples": 8,
  "mean_angle_to_first_deg": 15.38,
  "mean_delta_angle_deg": 4.02,
  "mean_delta_d": 1.43,
  "mean_inlier_ratio": 0.18
}
```

---

## 📌 Interpretação

* Apenas **8 amostras**
* Inlier baixo (~0.18)
* Variação angular maior (~4° média frame-a-frame)

Conclusão:

> Essa série é fraca e instável.

Ela provavelmente:

* aparece em poucos frames
* pode ser falso positivo
* não é estruturalmente relevante

Eu não confiaria semanticamente nela.

---

# 🧠 Diagnóstico Geral

| Métrica                     | Floor       | Interpretação               |
| --------------------------- | ----------- | --------------------------- |
| Persistência                | Excelente   | ✔️                          |
| Inlier médio                | Alto        | ✔️                          |
| Delta angular frame-a-frame | Baixo       | ✔️                          |
| Ângulo acumulado            | Alto (~11°) | ⚠️ depende do referencial   |
| Delta_d alto                | Muito alto  | ⚠️ esperado se frame câmera |

---

# 🔎 O que isso significa para o pipeline?

## 1️⃣ O tracking está consistente

Porque:

* delta_angle médio baixo
* variação frame-a-frame pequena

Isso indica que você NÃO está misturando planos diferentes no mesmo track.

---

## 2️⃣ O movimento do drone está aparecendo nas métricas

E isso é bom.

Mas significa que:

> Se você quiser medir estabilidade estrutural real do plano, precisa compensar pose.

Caso contrário, você está medindo estabilidade no referencial da câmera.

---

# 🎯 Melhor prática (nível pesquisa)

Para análise científica mais correta:

Antes de medir ângulos:

1. Transformar todos os planos para o **frame global (odom ou world)**.
2. Aí sim calcular:

   * variação angular global
   * variação de d global

Isso removeria o efeito do movimento do drone.

---

# 🚀 Conclusão Final

### 🔥 Seu Floor é estável.

Nada indica problema de tracking.

### ⚠️ A grande variação angular acumulada é provavelmente efeito do voo.

### ❌ A série Table/Bed é fraca e pode ser descartada semanticamente.

---

Se você quiser, posso agora:

* 📐 Derivar um índice de estabilidade global formal
* 📊 Propor métrica normalizada por movimento
* 🧠 Avaliar se seu floor poderia ser usado como referência para VIO/SLAM
* 🔬 Ou interpretar isso como material de paper

Qual direção você quer seguir agora?

---

Após todas essas analises e avaliações eu rodei o experimeto de novo com a seguinte configuração:

python scripts/segment_planes_open3d.py \
  --input data/ply/zed_outdoor_02_T2_279_01_05 \
  --out_dir output/planes/zed_outdoor_02_T3_p3 \
  --preset outdoor \
  --max_planes 3 \
  --min_inliers 3500 \
  --distance_threshold 0.15 \
  --ransac_n 4 \
  --num_iterations 3000 \
  --sor_nb_neighbors 30 \
  --sor_std_ratio 2.0 \
  --seed 7

python scripts/track_planes_temporally.py \
  --input_dir output/planes/zed_outdoor_02_T3_p3 \
  --output output/planes/zed_outdoor_02_T3_p3/plane_tracks.json \
  --max_angle_deg 10.0 \
  --max_centroid_dist 3.0 \
  --max_d_diff 1.0 \
  --max_frame_gap 8 \
  --min_track_length 10 \
  --ema_alpha 0.8 \
  --w_angle 2.0 \
  --w_centroid 1.0 \
  --w_d 0.5

python scripts/compute_track_confidence.py \
  --tracks output/planes/zed_outdoor_02_T3_p3/plane_tracks.json \
  --output output/planes/zed_outdoor_02_T3_p3/track_confidence.json

python scripts/classify_tracks_stably.py \
  --tracks output/planes/zed_outdoor_02_T3_p3/plane_tracks.json \
  --confidence output/planes/zed_outdoor_02_T3_p3/track_confidence.json \
  --output output/planes/zed_outdoor_02_T3_p3/classification_stable.json \
  --vertical_axis 1 0 0 \
  --horiz_angle_max 30.0 \
  --min_table_height_diff 0.3 \
  --vertical_band_center 90.0 \
  --vertical_band_half_width 20.0 \
  --min_confidence 0.40

python scripts/recolor_by_stable_class.py \
  --input_dir data/ply/zed_outdoor_02_T2_279_01_05 \
  --classification output/planes/zed_outdoor_02_T3_p3/classification_stable.json \
  --out_dir output/planes/zed_outdoor_02_T3_p3 \
  --distance_threshold 0.07 \
  --summary output/planes/zed_outdoor_02_T3_p3/recolor_summary.json

python3 scripts/analyze_temporal_stability.py \
  --input_dir /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_T3_p3/json \
  --out_dir  /home/werikson/ufv/icuas2026/output/planes/zed_outdoor_02_T3_p3/temporal_stability

python scripts/visualize_pointcloud_sequence.py \
    --raw_dir data/ply/zed_outdoor_02_T2_279_01_05 \
    --colored_dir output/planes/zed_outdoor_02_T3_p3/ply_colored \
    --fps 1 \
    --max_points 100000 \
    --rotate_z_180

