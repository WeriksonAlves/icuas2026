# Análise da branch main — ICUAS2026

Data: 15/10/2026. Commit analisado: `0818525` (`Update .gitignore to include specific PLY file patterns for outdoor scene data`).

## Parecer

A main contém um protótipo de pesquisa organizado para segmentar, rastrear e classificar planos em nuvens 3D. O núcleo do pipeline é idêntico ao da opt examinada anteriormente. Consequentemente, os problemas de identidade de experimentos, leitura binária ROS, coerência geométrica e interpretação temporal também estão presentes na main. Os resultados existentes não bastam para validar estabilidade de superfícies físicas nem superioridade de um sensor.

Os achados abaixo foram conferidos no código e nos artefatos da main. Resultados T1/T2/T3 de outdoor_02 e conteúdo de `paper` da opt não são usados como evidência experimental desta branch.

## Estado Git e comparação de branches

- Branch de origem: `opt`, commit `958b521`.
- Foram preservados em stash a modificação local de `.gitignore` e o relatório anterior não versionado, usando `git stash push -u`.
- Mensagem: `codex: preservar alteracoes da opt antes da analise main 2026-10-15`.
- Checkout atual: `main`, commit `0818525`, igual à referência local `origin/main`. Não foi feito fetch; essa igualdade não confirma o estado remoto atual.
- Nenhum stash foi aplicado à main. Nenhum código, dado ou histórico foi alterado nesta análise. A única adição na main é este relatório, ainda não commitado.

A comparação Git confirmou igualdade de todo `src/planes`, `configs`, `pyproject.toml`, README e das documentações `docs/docs.md` e `docs/planes_pipeline.md`. Nos scripts do pipeline, as diferenças estão no visualizador e na presença do inspetor de sequência na opt. Também mudam `.gitignore`, comandos úteis e artefatos experimentais.

| Item | main | opt examinada anteriormente |
| --- | --- | --- |
| Commit | `0818525` | `958b521` |
| Arquivos versionados | 5.334 | 8.673 |
| Python | 19 | 20 |
| PLY versionados | 3.595 | 5.840 |
| JSON versionados | 1.705 | 2.780 |
| PNG versionados | 8 | 23 |
| GIF versionado | Não | Sim |
| Pasta `paper` versionada | Não | Sim |
| Inspetor `inspect_ply_sequence.py` | Não | Sim |
| Exportação GIF e limites globais no visualizador | Não | Sim |

## Arquitetura efetiva

1. `ros_export.py`: assina PointCloud2, extrai XYZ/RGB, filtra/downsample e grava PLY.
2. `segmentation.py`: aplica SOR, extrai múltiplos planos com RANSAC, refina com SVD e calcula métricas.
3. `tracking.py`: associa detecções por ângulo, centroide e offset, usando matching guloso e EMA.
4. `track_confidence.py`: calcula índice heurístico de confiança.
5. `semantic_classification.py`: seleciona tracks globais para chão, mesa e parede, com filtro de confiança.
6. `recolor.py`: colore cada nuvem conforme distância aos planos médios desses tracks.
7. `temporal_analysis.py`: analisa rótulos locais dos JSON de segmentação, sem consumir a classificação estável.

Há uma implementação paralela em `classification.py`, sem confiança. A documentação apresenta essa API, mas a CLI ativa usa `semantic_classification.py`.

A separação biblioteca/CLI, as dataclasses e a persistência das métricas geométricas são pontos positivos. Não há implementação encontrada de calibração extrínseca, sincronização ZED–Livox, compensação de movimento/deskew, associação espacial entre sensores ou cálculo de erro cruzado.

## Achados prioritários

| Prioridade | Local | Problema e evidência | Ação recomendada |
| --- | --- | --- | --- |
| Alta | `scripts/run_planes_pipeline.py:157` | Apenas três componentes do nome definem a saída. T21 e T22 de ZED outdoor_01 geram a mesma pasta `zed_outdoor_01_p4`; colisão reproduzida na main. | Preservar ID completo e identificar configuração/execução. |
| Alta | `src/planes/segmentation.py:543` | `enumerate` substitui o ID original do frame; recolorização procura `frame_<índice>.ply`. Falha em sequências com lacunas ou início não zero. Os cinco conjuntos existentes não mostraram esse desencontro. | Propagar ID original e caminho de entrada entre etapas. |
| Alta | `src/planes/temporal_analysis.py:193` | Métricas usam classificação por frame, não tracks estáveis; podem misturar superfícies e eixos estimados diferentes. | Analisar por track e classificação estável, distinguindo diagnóstico local. |
| Alta | `src/planes/ros_export.py:87` | Dtype não completa padding até `point_step`, nem trata `row_step` e endianness na leitura. XYZ de 12 bytes com passo 16 reproduziu erro de buffer. | Usar offsets, itemsize, byte order e strides corretos. |
| Alta | `src/planes/tracking.py:225` | Ângulo ignora sinal, mas offset e EMA não alinham `(n,d)` com `(-n,-d)`. Duas observações do mesmo plano criaram dois tracks. | Alinhar sinal antes do custo, EMA e agregações. |
| Alta | `src/planes/recolor.py:84` | Plano médio de cada track é aplicado a todos os seus frames; geometria local é ignorada. Pode falhar com movimento em referencial do sensor. | Usar detecção por frame ou transformar explicitamente para referencial fixo. |
| Média | `scripts/compute_track_confidence.py:40` | Persistência divide por frames sobreviventes ao filtro de tracks, não por todos os frames da sequência. | Guardar total de entrada e frames sem tracks. |
| Média | `src/planes/segmentation.py` | Reexecução não remove arquivos excedentes: pasta reutilizada pode misturar execuções. | Diretório por execução e manifesto completo. |
| Média | `src/planes/geometry.py:27` | Convenção `nz >= 0` é frágil para normais com componente Z quase nula. | Orientação inicial robusta e alinhamento temporal. |
| Média | `src/planes/geometry.py:40` | Ajuste SVD não rejeita poucos pontos, colinearidade e degenerescência. | Validar quantidade, finitude e rank. |

## Evidência dos resultados presentes na main

| Experimento | Frames de segmentação | Tracks retidos | Frames no índice de tracks | Chão / mesa / parede: frames com rótulo estável |
| --- | ---: | ---: | ---: | --- |
| lab_01_T1_p8 | 50 | 31 | 43 | 3 / 3 / 5 |
| livox_outdoor_01_T21_p4 | 541 | 8 | 28 | 0 / 0 / 0 |
| livox_outdoor_01_T22_p4 | 275 | 5 | 18 | 0 / 0 / 0 |
| zed_outdoor_01_T21_p4 | 541 | 27 | 60 | 0 / 0 / 4 |
| zed_outdoor_01_T22_p4 | 275 | 16 | 56 | 3 / 5 / 0 |

São 1.682 JSON de segmentação nesses cinco conjuntos. Os demais JSON versionados incluem resumos e arquivos agregados.

A classificação local do laboratório identifica chão em 47 frames e parede em 48; a estável atribui esses papéis em apenas 3 e 5. Na ZED T22 há chão local em 89 frames e chão estável em 3. A distinção confirma que a análise temporal local não valida diretamente os papéis globais.

Os denominadores salvos de confiança são 43, 28, 18, 60 e 56, respectivamente. Por exemplo, no Livox T21 a persistência é calculada sobre 28 frames, embora tenham sido segmentados 541. Isso favorece os tracks sobreviventes e prejudica comparações entre experimentos.

As sequências Livox não têm nenhum dos três papéis globais selecionado. Isso pede investigação da geometria, eixo vertical, limiares e fragmentação; não prova qualidade inferior do sensor. Um PLY semântico gerado também pode ser totalmente cinza quando nenhum papel está ativo.

Reexecutei em memória o tracking de `zed_outdoor_01_T22_p4` com a configuração salva: foram obtidos os mesmos 16 tracks e as mesmas associações de frame e plano local. Essa consistência de execução não constitui validação da identidade física dos planos.

## Visualizador específico da main

`scripts/visualize_pointcloud_sequence.py` possui três limitações relevantes:

- Alinha listas de PLY pela posição e usa o menor comprimento, sem interseção de IDs. Arquivos ausentes podem causar comparação de instantes diferentes.
- `_downsample` usa `np.random.choice` sem semente por frame. Duas chamadas para a mesma nuvem retornaram amostras diferentes no teste isolado. Isso pode produzir cintilação e diferença aparente entre visualizações do mesmo dado.
- Recalcula limites por frame e por painel. Mudança de zoom/escala pode dificultar comparação de tamanho e movimento entre sensores e etapas.

A main não oferece as opções `--save_gif`, `--coordinate_frame`, `--limit_mode` e os controles de apresentação introduzidos na opt. Importar essas melhorias pode ajudar a apresentação, mas não corrige os problemas do núcleo compartilhado.

## Riscos científicos e limitações do método

O eixo vertical local é escolhido pelo suporte das normais aos eixos canônicos; isso não estima necessariamente gravidade. Uma parede dominante pode ser tomada como orientação horizontal. Na classificação estável, o default é `[1,0,0]`, exigindo validação por sensor/referencial e também do sinal para interpretar “acima”.

A escolha de apenas um track global por classe deixa de rotular fragmentos do mesmo chão. A seleção por suporte não garante distinguir chão de teto ou mesa dominante. O matching guloso depende da ordem das detecções; graus e metros entram no custo sem normalização pelos limiares.

O exportador não persiste timestamps, `frame_id` ou pose. Não é possível inferir do pipeline se as nuvens já chegam em referencial global. Sem essa informação, variação angular/offset pode refletir movimento do sensor, ruído, troca de superfície ou inversão de sinal. Deltas entre amostras disponíveis também podem atravessar lacunas de vários frames.

Confiança é um índice heurístico, não uma probabilidade calibrada. Baixa dispersão dos inliers não garante suporte alto ou classe correta. Estabilidade temporal não substitui avaliação de erro contra referência independente.

## Empacotamento, documentação e dados

O README contém apenas o link da documentação. Faltam instalação mínima, ambiente ROS/Python reproduzível e exemplo pequeno. A documentação anuncia suporte do tracking a JSON diretamente na pasta, mas o loader só procura `json/planes_*.json`; também diverge dos nomes/pastas reais da recolorização.

`configs/datasets.yaml` não é consumido pelo pipeline examinado. O orquestrador depende do diretório corrente e não expõe todos os parâmetros das etapas. Há duas classificações com comportamento e esquema diferentes (`plane_id`/`local_id` e confiança).

Não foram encontrados testes versionados, CI, lockfile ou LICENSE. O metadado MIT permanece comentado como placeholder no `pyproject.toml`. A configuração flake8 no TOML não é consumida pelo flake8 padrão, e o adaptador mencionado não está no extra dev.

A semente NumPy controla cores, sem semeadura explícita do RANSAC Open3D. O help limita corretamente a promessa à coloração. Falta registrar todos os parâmetros efetivos e versões num manifesto. As escritas PLY não verificam o retorno booleano; o exportador pode sobrescrever arquivos ao reiniciar e aguarda indefinidamente se o bag acabar antes da quantidade solicitada.

O Git contém 3.595 PLY e 1.705 JSON. O `.gitignore` da main cobre `data/ply/*home_*/` e `data/ply/outdoor*/`, mas deixa de fora nomes como `zed_outdoor_*` e `livox_outdoor_*`. A regra de outputs também é restrita a `home_*`. Arquivos já rastreados continuam versionados independentemente dessas regras. Recomenda-se separar amostra mínima, datasets e resultados pesados, mantendo checksums e procedência. Não foi feita remoção do índice ou reescrita de histórico.

## Verificações e limites

- Os 19 arquivos Python versionados passaram na análise sintática por AST.
- Os 1.705 JSON versionados foram lidos com rejeição explícita de NaN/Infinity: nenhum erro. Não é validação de esquema ou verdade científica.
- Foram reproduzidos na main a colisão de caminhos, a falha de padding PointCloud2, a fragmentação por sinal e a amostragem não determinística do visualizador.
- As associações dos 16 tracks ZED T22 foram reproduzidas com a configuração salva.
- O código compartilhado foi comparado por Git com a opt; a igualdade foi confirmada, sem pressupor igualdade dos artefatos experimentais.
- Não houve instalação de dependências, reprodução ROS, renderização GUI ou resegmentação dos PLY. Open3D está ausente e a importação de Matplotlib falha por incompatibilidade binária com NumPy no ambiente disponível, conforme verificado nesta sessão. Isso limita a execução integral; não é um teste realizado em ambiente limpo instalado a partir do projeto.
- Não houve auditoria geométrica integral dos PLY, pesquisa externa ou auditoria completa de segredos/vulnerabilidades.

## Sequência recomendada de correções

1. Preservar IDs completos de experimentos e frames; evitar reutilização ambígua de saídas.
2. Corrigir layout PointCloud2 e alinhamento de sinais, com regressões sintéticas.
3. Explicitar referencial, gravidade e timestamps; usar detecção por frame e métricas por track estável.
4. Corrigir persistência e reportar cobertura, fragmentação e frames sem detecção.
5. Consolidar classificação/configuração; reparar ambiente e documentar instalação e execução mínima.
6. Revalidar uma sequência pequena com referência manual antes de reprocessar datasets completos.
7. Para comparação ZED–Livox, implementar calibração/sincronização, matching espacial e protocolo com suporte/FOV comparáveis e referência independente.

Critérios de aceitação: caminhos sem colisão, IDs rastreáveis, planos equivalentes no mesmo track, exportação ROS fiel ao layout, métricas com população/referencial declarados e exemplo reproduzível em ambiente documentado.
