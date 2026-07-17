# Audit mapping / localisation MTT — 2026-07-11

Audit factuel de la stack ICP mapping/localisation. Aucune modification de code n'a
été faite pendant cet audit. Chaque affirmation « confirmé » a été vérifiée dans le
code ou le git ; les « probable » sont des hypothèses classées par vraisemblance.

---

## 1. État des lieux (confirmé)

### norlab_icp_mapper_ros — branche `mohamed`
- HEAD local `1dab75c` (« Stabilize replay mapper gating »), **1 commit en avance**
  sur `origin/mohamed`, non poussé.
- **Non commité : +1508 / −123 lignes** dans `mapper_node.cpp`, `NodeParameters.{h,cpp}`,
  `config/{_mapper,mapper}.yaml`. Ce diff contient toute la surface de paramètres
  `map recovery`, `motion adaptive gate`, `odom bridge`, `planar pose constraint`,
  `dynamic trailer self-filter`, `map_publication_source`.
- Écart vs `origin/ros2` (upstream) : **+4206 / −508 lignes** sur 12 fichiers.
  `mapper_node.cpp` : ~606 lignes upstream → ~3500 lignes chez nous.

### norlab_robot — branche `mtt-hl-devel`
- 1 commit en avance (`b66d50c`), non poussé.
- **Non commité : +342 / −16** dont `launch/include/icp_mapper.launch.py` (+156) et
  `launch/mapping.launch.py` (+80) : les arguments launch des features ci-dessus.
- `_config_hesai_wheel_replay.yaml` non commité : `updateCondition: distance → delay`,
  alors que le commentaire de `_config.yaml` (live) documente précisément pourquoi
  `delay` a été **reverté** (jusqu'à 8 m entre insertions en ligne droite).
  → Incohérence replay/live à trancher.

### ⚠️ Couplage critique non commité
`demos/bag_replay/compose.yaml` passe ~40 arguments launch qui n'existent **que**
dans le diff non commité de `norlab_robot`, lequel ne fonctionne qu'avec le diff non
commité de `norlab_icp_mapper_ros`. Un `git stash` ou checkout dans l'un des deux
repos casse silencieusement toute la stack replay. **À commiter en branche avant
tout test A/B.**

### Binaire périmé — CORRIGÉ après vérification (faux positif mtime)
Première lecture : source `mapper_node.cpp` modifiée 21:23 vs binaire `install/`
daté 18:07 → suspicion de binaire périmé. Vérification md5 : **le contenu
d'`install/` était identique au build frais** — le mtime d'`install/` est
trompeur (CMake préserve un horodatage ancien à la copie). Conclusion corrigée :
le binaire était à jour. `scripts/replay_bag.sh` embarque maintenant un check
fiable (source vs build par mtime, build vs install par contenu).

### libpointmatcher_ros — HEAD détachée `bdf5180` (= `origin/humble` HEAD, tag 2.0.1)
- `origin/fomo` vs HEAD actuel : **diff total de 25 lignes**, uniquement l'ajout du
  paramètre `bool isFomo = false` (valeur **par défaut**) sur les deux overloads de
  `rosMsgToPointMatcherCloud`. Effet : si `isFomo=true`, le champ temps FLOAT64 est
  interprété en µs (×1e3) au lieu de secondes (×1e9). Rien d'autre.
- **Conséquence : passer sur `origin/fomo` est rétro-compatible.** Tous les appels
  existants (`mohamed`, `ros2`) compilent sans changement, et `origin/mtt` (qui
  appelle `rosMsgToPointMatcherCloud(msg, isFomo)`) compile aussi. C'est le couple
  à utiliser si on veut tester `mapper:mtt`, comme supposé — confirmé dans le code.

### Zips de référence
- `src/norlab_icp_mapper_ros.zip` = branche `ros2` à `0e443a0` = **HEAD exact de
  `origin/ros2`**. C'est le mapper upstream stock (« config Mathis/base »).
- `src/norlab_robot.zip` = `mtt-hl-devel` à `348ae00` = **le parent direct du commit
  actuel**. Ce zip n'est PAS une référence indépendante « Mathis » : c'est notre
  propre repo moins un commit. Seul le zip mapper est une vraie référence.

### imu_odom — branche `altimeter-icp`
- Ajoute `use_altitude` (souscription altimètre, correction z). Dernier commit
  « upgraded the use_altitude=false way to work ». Replay : `use_altitude=false`,
  `rotation_only=true` (profil `imu_odom`) / `rotation_only=false` (profil
  `mathis_mapping`, intégration accéléromètre + recalage par `/mapping/icp_odom`).

### Garde-fous déjà en place (à ne pas ré-inventer)
- `scripts/detect_tf_conflict.py` + `MAPPING_TF_CONFLICT_CHECK=true` : le mapper
  **refuse déjà de démarrer** si yaw jump > 25° sur `odom → base_footprint`
  (le cas runtime_odometry + imu_odom simultanés, jumps ~89° observés).
- Le mapper `mohamed` publie déjà un statut par scan sur `/mapping/status`
  (`diagnostic_msgs/DiagnosticStatus`, timer + par-scan) : la base des métriques A/B
  existe déjà.

---

## 2. Base Mathis (zip `ros2`) vs stack `mohamed` — le différentiel qui compte

| Paramètre / mécanisme | Base zip (`ros2`) | Stack MTT (`mohamed` + configs) |
|---|---|---|
| mapper_node | 606 lignes | ~3500 lignes |
| Insertion carte | `delay: 0.5 s`, **toujours** | gates overlap ×4 + correction max + freeze + recovery |
| Gates pose | aucune | yaw step, odom residual, z jump, vitesse, temps ICP |
| KDTree `maxDist` | **200 m** (quasi illimité) | **1.0 m** (`_config.yaml`) |
| Échantillonnage lecture | RandomSampling 0.8 | 0.25 |
| `force4DOF` | **1** (roll/pitch = prior) | **0** (6-DOF libre) dans `_config.yaml` ET `mapper_snow_no_trailer.yaml` |
| BoundTransformationChecker | absent | présent |
| Itérations max | 40 | 20 |
| Deskew | **off** | on, `source=imu`, `absolute_ns` |
| Frame robot | `base_link` | `base_footprint` |
| Carte locale/globale | une seule carte | split local/global + trim + recovery |
| Self-filter trailer | aucun | bbox statique + OBB dynamique articulé |

### Pourquoi la base marche si bien en replay ×1 (diagnostic)

1. **Elle ne peut pas se deadlocker.** Aucune gate : chaque scan est inséré, la TF
   est toujours publiée. À vitesse réelle avec un prior correct, ICP converge et la
   carte paraît propre. Notre stack, elle, peut entrer dans le cercle vicieux
   documenté dans `compose.yaml` (lignes 560-564) : zone nouvelle → insertion
   refusée par la gate overlap → carte gèle → overlap baisse encore → tous les
   scans rejetés, toujours au même endroit du bag. C'est la cause n°1 des
   « cassures » reproductibles.
2. **`maxDist: 200` tolère un prior très faux.** Sur neige avec patinage, le prior
   roue est faux de 2-3 m entre updates ; avec `maxDist: 1.0` les correspondances
   disparaissent et l'ICP est affamé, puis rejeté par les gates. La base retrouve
   des correspondances quand même (au prix d'un risque d'appariement faux à grande
   distance — d'où son drift, invisible sur un petit trajet).
3. **`force4DOF: 1` supprime le z-drift/roll-pitch.** Nos deux configs actives
   (`_config.yaml`, `mapper_snow_no_trailer.yaml`) sont en 6-DOF (`force4DOF: 0`)
   alors que leur propre commentaire dit « sur terrain plat : force4DOF=1 ». Sur
   neige plate sans structure verticale, le 6-DOF laisse dériver roll/pitch/z →
   murs doubles, swirl. C'est le levier A/B le plus simple et le plus probable.
4. **Deskew off = zéro risque de timestamp.** Le Deskewer upstream (77 lignes)
   construit `rclcpp::Time(cloud.times(i))` directement : il exige des temps
   **absolus en ns** et une TF odom couvrant l'intervalle du scan. Tout écart de
   convention (temps relatif, TF en retard en replay) produit les erreurs
   « Requested time -129153… » observées sur la branche `ros2` avec
   `MAPPING_DESKEW=true`. Confirmé par lecture du code. Notre branche a corrigé ça
   (`deskew_time_mode=absolute_ns`, `deskew_source=imu`) — mais le deskew IMU
   reste un point de fragilité à valider par A/B, pas à présumer correct.

### Pourquoi notre stack casse (classement)

| Cause | Statut |
|---|---|
| Deadlock gates/freeze insertion (zone nouvelle) | **Confirmé** (documenté + observé, mécanisme lisible dans le code) |
| Binaire pas recompilé après édition C++ | **Confirmé aujourd'hui** (source 21:23 > binaire 18:07) |
| Conflit TF double odom (89° yaw jumps) | **Confirmé, déjà gardé** par detect_tf_conflict.py |
| `maxDist: 1.0` + prior roue faux sur slip → ICP affamé | **Très probable** (à mesurer : taux de rejet vs erreur prior) |
| 6-DOF (`force4DOF: 0`) sur terrain plat → z/roll/pitch drift | **Probable** (contredit ses propres commentaires de config) |
| Deskew IMU : signe/axe/latence gyro, convention temps par bag | Possible — à valider par A/B deskew off/TF/IMU |
| Gate rotation insertion relâchée → accumulation yaw 90° | **Confirmé historiquement** (commentaire compose ligne 572-574), déjà remis à 12° |
| OBB trailer dynamique masquant trop de points arrière | Possible en config trailer ; neutralisé dans mathis (`false`) |

---

## 3. Matrice de tests A/B

Préalables obligatoires :
1. Commiter les diffs non commités (mapper + norlab_robot) sur branches dédiées.
2. `docker compose run --rm compile` et vérifier `stat install/.../mapper_node`.
3. Même bag, même vitesse (×1), un seul facteur variable par run.

| # | Mapper | libpointmatcher_ros | Odom TF | Deskew | Config ICP | But |
|---|---|---|---|---|---|---|
| A1 | `ros2` (zip/base) | `bdf5180` | runtime_odometry | off | mapper.yaml stock | référence « ça marche » |
| A2 | `mohamed` | `bdf5180` | runtime_odometry | **off** | `_config_hesai_wheel_replay` | isoler le deskew |
| A3 | `mohamed` | `bdf5180` | runtime_odometry | imu | idem | valider deskew IMU |
| A4 | `mohamed` | `bdf5180` | **imu_odom** (sans `--profile mapping`, cmd explicite) | imu | `_config_hesai_imu_replay` | prior IMU vs roue |
| A5 | `mohamed` | `bdf5180` | runtime_odometry | imu | idem + **force4DOF: 1** | z-drift / murs doubles |
| A6 | `mohamed` | `bdf5180` | runtime_odometry | imu | idem + **maxDist: 2.0–3.0** | slip → ICP affamé |
| A7 | `mohamed`, **gates off** (`MAPPING_ENABLE_MOTION_ADAPTIVE_GATE=false`, overlap gates à 0, recovery off) | `bdf5180` | runtime_odometry | imu | idem | prouver/disculper le deadlock de gates |
| A8 | `mtt` | **`origin/fomo`** | runtime_odometry | off | config mtt | curiosité seulement — branche ancienne, pas un candidat prod |
| B1/B2 | meilleur candidat | — | idem | idem | avec vs sans trailer (OBB on/off) | coût du self-filter |
| C1/C2 | meilleur candidat | — | idem | idem | bag garage vs bag neige/patinoire | sensibilité terrain |

Commande replay propre imu_odom (rappel, sans `joint_state_builder` ni `runtime_odometry`) :

```bash
BAG_PATH=... docker compose up bag_player description imu_odom mapping foxglove -d
```

## 4. Métriques (collecte via `/mapping/status`, déjà publié par scan)

À logger en CSV par run (petit script `ros2 topic echo → csv`, ou souscripteur dédié) :
- temps de registration par scan (ms), taux d'acceptation ICP (% scans acceptés)
- overlap near/loose par scan
- norme de correction ICP translation (m) et yaw (deg) par scan
- drift de fermeture : distance entre pose de départ et pose au retour au même
  endroit (le bag patinoire revient au point de départ → mesurable au mètre)
- cohérence ZED VO (`REPLAY_KEEP_ZED_ODOM=true`) et GPS/RTK si présents dans le bag :
  erreur relative de trajectoire, pas seulement visuel
- qualité Foxglove : murs doubles, swirl, yaw jumps (checklist binaire par run)

Critère de décision : un changement n'est retenu que s'il améliore une métrique
mesurée sur le même bag, à facteur unique.

---

## 5. Architecture cible localisation (proposition)

Principes :
- **Le mapper ICP n'est pas la vérité.** C'est une odométrie LiDAR incrémentale
  sans loop closure. Il produit : `/mapping/icp_odom` + métriques + covariance.
- Aujourd'hui `icp_odom` part avec une **covariance nulle (non renseignée)** —
  c'est le premier trou à combler (covariance estimée depuis overlap, résidu ICP,
  nb de correspondances ; même une heuristique 3 niveaux honnête vaut mieux que 0).
- **Fusion dans un backend séparé** (nouveau package, ex. `mtt_fusion`, hors
  `mapper_node.cpp`) : factor graph (GTSAM) fusionnant IMU (preintégration),
  odom roue/tacho, ICP odom, ZED VO, GPS/RTK si dispo.
  - Slip détecté (incohérence tacho vs IMU/ICP) → covariance roue gonflée fortement.
  - ICP faible (overlap bas, correction énorme) → covariance ICP gonflée ou facteur rejeté.
  - ZED faible (neige uniforme, tracking lost) → covariance VO gonflée.
  - Sortie unique : `map → odom` + `/localization/odom`.
- Étapes incrémentales (pas de big-bang) :
  1. Covariance honnête sur `icp_odom` (petit patch mapper, mesurable).
  2. Logger CSV de métriques + campagne A/B (section 3).
  3. Prototype fusion offline sur bags (Python/GTSAM) — valider le gain avant
     tout nœud temps réel.
  4. Nœud fusion online seulement si le gain offline est démontré.

## 6. Loop closure

- **Jamais dans `mapper_node.cpp`** (déjà 6× la taille upstream).
- Package/backend séparé : place recognition LiDAR (Scan Context) → candidat →
  **validation ICP stricte** (overlap + résidu) → BetweenFactor dans le pose graph.
- Online : prudent (seuils stricts, budget CPU borné). Offline : batch agressif
  pour produire la trajectoire de référence.

## 7. Vocabulaire ground truth

- Trajectoire ICP/pose-graph offline = « **référence optimisée offline** », pas
  ground truth. Objectif : erreur relative ≤ 5 % sur métriques mesurables.
- Vrai ground truth uniquement : RTK fix fiable, station totale, mocap, landmarks
  mesurés, ou fermeture de boucle mesurée au ruban.

---

## 8. Recommandations

### À faire (petit, réversible, dans l'ordre)
1. **Hygiène repo (avant tout test)** : commiter les diffs non commités mapper +
   norlab_robot sur branches dédiées, pousser `mohamed`. Trancher l'incohérence
   `updateCondition` delay/distance entre replay et live.
2. **Rebuild systématique** : ajouter au script de lancement replay un check
   « binaire plus récent que la source » (comparaison mtime, warning sinon) —
   ~15 lignes dans `scripts/replay_bag.sh`.
3. **libpointmatcher_ros → `origin/fomo`** : 25 lignes, rétro-compatible, débloque
   la compilation de `mapper:mtt` pour les A/B. Risque quasi nul.
4. **Campagne A/B section 3** avec logger CSV des métriques `/mapping/status`.
   Les leviers à tester en premier : `force4DOF: 1`, `maxDist` 1.0→2.5,
   gates off (A7), deskew off/imu.
5. **Covariance honnête sur `icp_odom`** (heuristique depuis overlap/résidu).
6. Profils Docker A/B propres si la matrice devient récurrente (un profil par
   ligne de la matrice, au lieu d'empiler des env vars à la main).
7. Configs séparées assumées : `mathis/no-trailer` vs `data_collection/trailer`
   (déjà en place — les garder divergentes et documentées, ne pas re-fusionner).

### Correctifs appliqués (loop du 2026-07-11, working tree, non commités)
1. **Deskew IMU — intégration par morceaux** (`Deskewer.cpp::deskewCloudImu`) :
   l'ancien modèle appliquait UN échantillon gyro en ω constant sur tout
   [t_point, fin de scan] — faux dès que ω varie pendant le scan. Remplacé par une
   rétro-intégration trapézoïdale des échantillons IMU (A(t_k)=A(t_{k+1})·R(−ω̄·Δt)).
   Validé par self-test synthétique (`.deskew_selftest/`) : rampe 11→115 °/s sur un
   scan de 100 ms à 6 m → résidu 0,54 m (ancien modèle) → 0,0001 m (nouveau).
2. **Covariance honnête sur `/mapping/icp_odom`** (`mapper_node.cpp`) : pose et
   twist covariance dérivées de la correction ICP du scan ; ×3 si acceptation par
   gate relâchée (adaptative/cascade) ; σ_xy = 0,30 m + 0,5·v·dt si odom-bridge
   (dead reckoning) ; twist angulaire marqué non estimé (1e3).
3. **Check fraîcheur binaire** dans `scripts/replay_bag.sh` : source vs build par
   mtime, build vs install par contenu (les mtimes d'`install/` sont trompeurs).
Rebuild Docker OK, smoke-test de démarrage du nœud OK.

### Correctifs appliqués — suite (2026-07-12)
4. **Deskew hybride** (`Deskewer.cpp` + `mapper_node.cpp`) : le mode `imu` était
   rotation-seule → smear de translation ~v·0,1 m (0,2 m à 2 m/s). Ajout d'une
   compensation translation à vitesse constante dérivée de la TF odom sur la
   fenêtre du scan (erreur résiduelle = erreur de vitesse odom × 0,1 s, bien
   plus petite que le smear complet). Fallback rotation-seule si TF indispo.
5. **Harness synthétique** `.synth_replay/` : bag MCAP à vérité connue (slip
   linéaire 35 % + slip yaw 35 % au pivot), éval ATE/dérive/yaw/acceptation,
   modes norlab (deskew off/tf/imu) et kiss. Résultats bag durci :
   deskew off 9,55 % de dérive ; tf 0,04 % ; **imu hybride 0,05 % (ATE 0,023 m,
   le meilleur)** ; KISS-ICP 2,74 % (26× pire en ATE que le mapper).
   Le choix `deskew_source=imu` de la stack est donc validé ET amélioré :
   l'hybride garde l'immunité au slip yaw et récupère la précision translation.

### Validation sur bag réel — patinoire 2026-07-01_11-46-15 (calib v2, 546 s)

Trajectoires comparées (extraction + replays frais, CSV dans `.synth_replay/out_0701/`) :

| Trajectoire | Chemin | Fermeture start↔end |
|---|---|---|
| ICP live de la session | 1131 m | 29,98 m |
| /mtt_odometry (roues) | 1141 m | 30,69 m |
| ZED odom (slip-immune) | 710,8 m | 2,20 m |
| ICP frais, prior roue (mapper corrigé) | 1077 m | 8,85 m |
| **ICP frais, prior imu_odom (mapper corrigé)** | **685,9 m** | **6,60 m** |

Conclusions :
1. Sur glace, l'odom roue surestime le chemin de ~60 % (patinage). L'ICP live de
   la session l'a suivie au lieu de la corriger (fermeture 30 m).
2. Le mapper corrigé avec prior roue divise la fermeture par 3,4 (8,85 m) mais
   les logs montrent des cascades odom-bridge pendant les rafales de slip : la
   gate de plausibilité ancrée sur l'odom rejette l'ICP correcte quand l'odom
   ment (roue à 4 m/s vs 1,3 m/s réels) et publie le prior roue à sa place.
   **Sur glace, la roue ne peut pas servir de référence de plausibilité.**
3. **Prior imu_odom : chemin à 3,5 % de la ZED, fermeture 0,96 % du chemin.**
   C'est la config recommandée sur terrain glissant — le demo mathis_com_shift
   l'avait déjà adoptée ; c'est maintenant mesuré sur bag réel.
4. Bug corrigé au passage : buffer gyro du deskew non trié après wrap du bag
   (`--loop`) → reset sur saut temporel arrière.
5. Config `_config_hesai_imu_replay` lourde (~1,8 Hz traité sur 10 Hz) — encore
   OK en replay ; pour du live IMU-prior, reprendre la chaîne de filtres légère
   du config wheel.

### Validation sur bag réel — 2026-06-02_09-01-53 (calib v1, 2170 s)

Bag long : malgré le nom de session « test_garage », la carte assemblée révèle
un **circuit extérieur ~500×400 m** (rues bordées d'arbres), 2,7 km en 36 min.
Références partielles : ICP live morte après 90 s ; ZED avec 74 m d'amplitude z
(en partie du vrai dénivelé) ; roue et ZED d'accord sur la longueur
(2715 vs 2778 m) mais la roue diverge de ~100 m en absolu.

**ERRATUM (2026-07-12)** : le premier replay frais (48,5 % acceptés, chemin
amputé à 1427 m, 60 sauts publiés de 5-13 m, téléportations ICP 70-80 m
rejetées) avait été diagnostiqué « aliasing de revisite ». Faux : un conteneur
`imu_odom` **résiduel d'un run précédent** publiait une TF `odom→base_footprint`
concurrente de `runtime_odometry` pendant tout le run (les `docker compose down`
ne tuent pas toujours les services `up -d` — purger par
`docker ps | grep bag_replay- | xargs docker rm -f`).

**Rerun propre** : **86,6 % acceptés, ~13 Hz, chemin 2724 m** (= roue 2715,
ZED 2778), **zéro téléportation**. Le mapper corrigé suit tout le mouvement.
Problème restant réel : dérive yaw cumulée sur 36 min → fermeture 128 m (4,7 %)
— exactement le domaine du pose graph/loop closure, pas du tuning de gates.
Faux positif corrigé au passage dans `detect_tf_conflict.py` : le saut d'init
du cap IMU (114,7°) n'est pas un conflit de publishers (`--settle 5 s`,
`--min-alerts 3`).

### Pose graph + loop closure — prototype validé (2026-07-12)

`.synth_replay/pose_graph_proto.py` sur le bag patinoire :
1. Nœuds = odométrie ICP fraîche (config imu lourde, 686 m), sous-échantillonnés
   au mètre, pondérés par la **covariance honnête** du mapper.
2. Ancres toutes les ~45 s → submaps de 5 scans recalés par la trajectoire.
3. Place recognition : corrélation FFT de grilles d'occupation (translation
   libre, yaw borné ±30° autour du prior — évite l'alias 180° de la patinoire).
4. Validation à deux niveaux : forte (rms ≤ 0,10 m ET inliers ≥ 75 %) acceptée
   d'office ; modérée soumise à un budget de déviation. Les tentatives mal
   initialisées avaient été correctement rejetées avant l'ajout du coarse FFT.
5. Optimisation SE2 (scipy least_squares).

Résultats : **14 boucles validées** (rms 0,018–0,18 m), résidu médian après
optimisation **0,137 m** sur 686 m. Vérité mesurée par la boucle start↔end
(rms 1,8 cm, 95 % inliers) : le robot a fini à **0,54 m** de son départ — la
dérive ICP brute était 6,6 m / 21,4°, et la ZED elle-même avait ~2 m d'erreur
sur son écart annoncé (2,20 m). Livrables :
`out_0701/optimized_trajectory{,_multi}.csv` — « référence optimisée offline »
(pas ground truth : erreur relative résiduelle ~dm entre ancres).

### Verdict des deux bags réels
| Bag | Conditions | Stack live (session) | Mapper corrigé |
|---|---|---|---|
| Patinoire 07-01 (v2) | slip extrême | fermeture 30 m | **prior IMU : 6,6 m brut → réf. offline 14 boucles (méd 0,137 m)** |
| Garage 06-02 (v1) | 36 min, 2,7 km de boucles | ICP morte à 90 s | 86,6 % acceptés, chemin complet, fermeture 128 m (dérive yaw) → pose graph |

Priorités qui en découlent : (1) prior IMU par défaut sur terrain glissant,
(2) calibration extrinsèque propre + hygiène conteneurs au replay,
(3) backend loop-closure séparé pour les longues sessions — prototype
`.synth_replay/pose_graph_proto.py` validé (fermeture incrémentale SLAM-style).

### À éviter
- Ajouter d'autres gates/features dans `mapper_node.cpp` avant que la campagne A/B
  ait montré quelle moitié des gates existantes est réellement utile. Le deadlock
  observé vient de l'**interaction** des gates, pas de leur absence.
- Loop closure dans le mapper.
- Relâcher `max_map_update_rotation_correction_deg` au-delà de 12° (accumulation
  yaw → 90° déjà observée et documentée).
- Toute feature sans métrique mesurable associée (règle des 500 lignes).
- Considérer la branche `mtt` comme candidat prod : base ancienne, elle a servi
  d'inspiration mais `mohamed` est plus à jour.
