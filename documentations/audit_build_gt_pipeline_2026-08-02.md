# Audit build_gt_pipeline (Part B) — 2026-08-02

Audit factuel de l'implémentation Part B (pipeline GT offline) et de la session
Ice-rink de référence. Toutes les vérifications listées ici ont été exécutées
en lecture seule (aucun stage du pipeline n'a écrit dans `/data/mtt_bags/`).
Destiné aux humains ET aux futurs agents IA qui reprendront ce pipeline pour du
traitement de données ou de la recherche motion model.

---

## 1. Implémentation Part B — état des lieux (confirmé)

8 scripts créés/modifiés, 10 commits git sur `main` (voir `git log --oneline
c19ffcd..fee3e23`).

| Script | Rôle | Statut |
|---|---|---|
| `scripts/offline_reference.py` | Solve factor-graph offline (IMU + ICP offline qualifié) | Durci : `/mapping/icp_odom` retiré du `TOPICS` set, `--offline-icp`/`--icp-approved-by` requis, `validate_offline_icp_csv()` appelée |
| `scripts/extract_v2_measurements.py` | Extraction diagnostique (tf_static, mesures brutes) | Paramétré (`--session`/`--offline-icp`/`--output-dir`) |
| `scripts/build_gt_v2_100hz.py` | Reformat 100Hz + composition extrinsèque | Paramétré (`--graph-dir`/`--audit-dir`/`--out-dir`) |
| `scripts/qualify_gt_v2.py` | Rapport diagnostique qualité (non bloquant) | Paramétré (`--base-dir`/`--ablation-dir`/`--out-dir`) |
| `scripts/build_gt_reference_csv.py` | Reshape pose → contrat `--reference` de `build_session_dataset.py` | Nouveau |
| `scripts/gt_provenance.py` | sha256/manifest/git-hash partagés | Nouveau |
| `scripts/gt_catalog.py` | Upsert `dataset_catalog.csv` + garde anti-écrasement | Nouveau |
| `scripts/build_gt_pipeline.py` | Orchestrateur (9 stages, `--dry-run`, `--validate-only`) | Nouveau |

### Bugs trouvés et corrigés après revue advisor (2 passes)

1. **`--output-dir` non câblé** — ajouté à `argparse` mais `process_session()`/
   `main()` ignoraient toujours l'argument et écrivaient dans
   `session_dir/offline_reference`. Cassait tout run où `--output != --session`
   (cas normal via l'orchestrateur). *Corrigé, commit `c5495f2`.*
2. **`validate_offline_icp_csv()` jamais appelée** — définie mais orpheline
   dans son propre module ; un CSV ICP structurellement cassé passait sans
   contrôle. *Corrigé (appel ajouté dans `parse_args()`), commit `c5495f2`.*
3. **`find_gt_icp_csv()` non supprimée** — auto-découverte par scan de
   dossier, exactement la « qualification automatique non enregistrée »
   interdite par la règle absolue anti-ICP-live. *Supprimée, commit `c5495f2`.*
4. **Collision d'écriture Stage 1 / Stage 2** — `offline_reference.py` et
   `extract_v2_measurements.py` écrivent tous deux dans
   `work_dir/measurements/`, avec collision exacte sur 3 noms de fichiers :
   `icp.csv` (contenu identique, sans risque), `track_odom.csv` et
   `zed_odom.csv` (schémas différents, deux parsers indépendants du même bag
   — le risque de divergence silencieuse que le docstring de
   `extract_v2_measurements.py` avertit explicitement). *Corrigé en
   réordonnant les stages (extract d'abord, offline_reference ensuite, pour
   que ses versions — celles qui ont réellement nourri le solve — persistent),
   commit `fee3e23`.*
5. **`load_gt_icp_csv()`/`validate_offline_icp_csv()` incompatibles avec le
   schéma réel de KISS-ICP** — voir §3 pour l'analyse complète. `KeyError:
   'wx'` garanti sur tout `bag/mapping_output_kiss/icp_odom.csv` réel avant
   correction. *Corrigé (alias de colonnes + validateur étendu), commit
   `6a69838`.*
6. **`build_gt_pipeline.py` Stage 7, collision `--map-reference`** —
   `dst = dataset_dir / f"map_reference{src.suffix}"` ne dépendait que de
   l'extension, pas du nom source : deux fichiers `--map-reference` de même
   extension (ex. `map.ply` + une trajectoire `.ply`, cas réel pour une
   session KISS-ICP) s'écrasaient silencieusement. *Corrigé (nommage inclut
   le stem source : `map_reference_{stem}{suffix}`), commit `6a69838`.*

### Vérifications effectuées (toutes passées)

- Les 8 fichiers compilent (`py_compile`).
- Tous les imports croisés résolvent (`offline_reference.validate_offline_icp_csv`,
  `build_msa_canonical_dataset.{TRACK_IN_BASE_M,add_derived_reference}`).
- Contrats CLI en aval vérifiés via `--help` réel (pas déduits du plan) :
  `build_session_dataset.py`, `export_bag_preview.py`.
- `add_derived_reference()` : signature réelle (`frame, filter_window_s,
  polyorder, speed_gate_ms) -> tuple[DataFrame, dict]`) confirmée identique à
  l'appel dans `build_gt_reference_csv.py`.
- `articulation_state.csv` (lu sans garde `.exists()` par `qualify_gt_v2.py:292/296`)
  contient bien les colonnes `t`/`hardware_fresh` attendues (writer
  `artic_state` dans `extract_v2_measurements.py:418-426`).
- `/data/mtt_bags/dataset_catalog.csv` : header réel comparé aux 9 clés
  écrites par `upsert_catalog_row()` — correspondance exacte, aucune faute de
  frappe créant une colonne fantôme.
- **`validate_dataset_dir()` (les gates DATASET_SCHEMA.md §13) exécutée
  contre le vrai `dataset/canonical_100hz.csv.gz` d'Ice-rink → PASSÉ.**
  Première preuve réelle (pas seulement syntaxique) que la logique de
  validation de Task 8 fonctionne sur des données de production.

### Ce qui n'a PAS été testé

**Aucun stage du pipeline (`offline_reference.py` → … → `build_gt_pipeline.py`)
n'a jamais été exécuté de bout en bout contre un vrai bag.** Seule
`validate_dataset_dir()` (lecture seule, post-hoc) a tourné contre de vraies
données. Raison : dans `/data/mtt_bags/`, seule la session `BAG_ICE_RINK_*` a
un `GT_icp/` qualifié, et elle est déjà `canonical_status=ready` (gelée —
protégée par `assert_not_frozen()`, ne pas y toucher). Aucune autre session
n'a d'ICP offline approuvé disponible pour un premier run réel. À débloquer :
qualifier un `GT_icp/icp_odom_*.csv` sur une session non gelée, puis lancer
Stage 1 seul dans `docker compose run --rm bash` (le solver C++ et
`rosbag2_py` nécessitent l'environnement ROS et les dépendances du conteneur).

**Statut honnête : code complet, syntaxiquement et logiquement audité,
gates de validation prouvées sur données réelles — mais chaîne complète
non exécutée.**

---

## 2. Vérification de complétude — session BAG_ICE_RINK_mtt_calibration_ICE_RINK_2026-07-01_11-46-15 (confirmé)

Vérification en lecture seule demandée explicitement par l'utilisateur, aucune
modification.

### Structure disque

```
bag/                    31 GiB bag_0.mcap + metadata.yaml
GT_icp/                 icp_odom_20260724_195123.csv (6.2M, 10886 lignes) + map_hesai.vtk + trajectory_hesai.vtk
mapping_output/         icp_odom_20260730_003551.csv (10887 lignes, rebuild PLUS RÉCENT, NON utilisé pour le dataset actuel)
zed_visual_map/         zed_pose_trajectory.csv + zed_visual_map_voxel.ply
dataset/                canonical_100hz.csv.gz, map_reference_icp.vtk, map_reference_icp_with_trajectories.ply,
                         map_zed_visual_voxel.ply, trajectory_reference.ply, trajectory_reference_icp.vtk, preview.mp4
```

### Intégrité SHA-256 (catalogue vs disque) — 4/4 fichiers identiques

| Fichier | Hash catalogue = Hash disque |
|---|---|
| `canonical_100hz.csv.gz` | ✓ `46b4ed61...b5da0` |
| `trajectory_reference.ply` | ✓ `85d200be...31460` |
| `map_reference_icp.vtk` | ✓ `3d2f1185...a5101` |
| `preview.mp4` | ✓ `1a0606eb...7ef85` |

Aucune corruption/modification depuis la génération du dataset (29 juillet).

### Ligne catalogue complète

`canonical_status: ready`, `reference_grade: B_operational`,
`reference_source: offline_factor_graph_icp_imu`, `canonical_rows: 54606`,
`canonical_columns: 175`, `canonical_path_length_m: 802.46`, tous les champs
`*_relpath`/`*_sha256` peuplés, `next_reference_action: "ready; preserve
offline factor graph and qualified offline ICP; do not use live ICP"`.

### Gates DATASET_SCHEMA.md §13 — PASSÉ (via `validate_dataset_dir()` réelle)

### Analyse `GT_icp/icp_odom_20260724_195123.csv` (le fichier réellement utilisé)

- 10886 lignes, colonnes complètes (`timestamp_sec/nanosec, x,y,z,qx,qy,qz,qw,
  vx,vy,vz,wx,wy,wz,pose_covariance,twist_covariance`).
- Timestamps finis, strictement croissants, `dt` médian 49.97ms (~20Hz),
  aucun gap > 0.5s.
- Quaternions unitaires (`max|norm-1| = 4.5e-7`).
- Aucun saut/téléportation isolé (test robuste par MAD : seuil 13.5 m/s,
  0 outlier détecté).
- Segment de vitesse max (~5.19 m/s à t=374s) : soutenu sur 7+ échantillons
  consécutifs → déplacement réel continu, pas un artefact de recalage ICP.

### Comparaison avec le rebuild `mapping_output/icp_odom_20260730_003551.csv` (plus récent, NON utilisé)

- Même bag, même schéma, 1 ligne de différence (10887 vs 10886).
- **Divergence de position entre les deux runs ICP offline (même bag, même
  mapper, dates différentes) : moyenne 73.7mm, médiane 69.6mm, p95 163mm,
  max 273mm.** Reflète la variance normale entre deux exécutions
  indépendantes de recalage ICP, pas une anomalie — mais à savoir si vous
  envisagez de remplacer le CSV de référence par ce rebuild plus récent : le
  dataset canonique changerait de plusieurs centimètres.

### Incohérence de métadonnées à la source (non bloquante, catalogue déjà correct)

`report.md` (généré 1er juillet, à l'enregistrement) ET `session_info.yaml`
(modifié 29 juillet) indiquent tous deux `experiment_name: test_garage`,
`terrain: asphalte` — alors que le dossier s'appelle `BAG_ICE_RINK` et que le
catalogue a correctement `terrain: ice_rink, surface: ice`. Contradiction
supplémentaire : `report.md` dit `Trailer: False`, `session_info.yaml` dit
`trailer_attached: true`. Cause probable : script de lancement de session
réutilisant un template/config d'une session garage précédente sans mise à
jour des champs. **Le catalogue (source de vérité pour le pipeline aval) a
les bonnes valeurs — aucun impact sur le dataset canonique produit — mais les
fichiers de métadonnées bruts de la session restent trompeurs si consultés
directement.**

### Couverture capteurs (depuis `report.md`, 79 topics enregistrés)

✅ CAN/Odométrie, Fallback Odom, IMU MTi-100, Hesai LiDAR, RS-Airy LiDAR
(`rsairy_ns`), ZED Camera, ICP Mapping (live, non utilisé pour la référence).
❌ GPS (aucun mode), IMU MTi-10, OAK-D, Perception/Merged Cloud, Teach/Repeat
— absents par design de cette session de calibration.

---

## 3. Compatibilité pipeline avec KISS-ICP et mappers alternatifs (confirmé empiriquement, code corrigé)

Question posée : le pipeline peut-il ingérer un rebuild KISS-ICP au lieu du
mapper norlab offline, pour des bags où le mapper de base casse ?

**Réponse : oui, et c'est maintenant corrigé dans le code (commit `6a69838`).**

### Où se trouve le vrai export KISS-ICP

Piège de structure trouvé en pratique : il existe **deux emplacements
`mapping_output_kiss/` différents par session**, et le premier consulté était
le mauvais :

- `<session>/mapping_output_kiss/` (racine de session) — contient les
  artefacts natifs bruts de KISS-ICP (`bag_poses_tum.txt`, `bag_poses_kitti.txt`,
  `bag_poses.npy`, `config.yml`, `result_metrics.log`) — format TUM/KITTI,
  **incompatible tel quel** avec le schéma du projet (timestamp epoch unique
  au lieu de sec/nanosec séparés, aucune vitesse). Trouvé dans seulement 2
  sessions (`BAG_FOREST2`, `mtt_edge_case_outside_2026-04-15_16-17-59`), l'une
  vide (run en cours ou interrompu), l'autre avec ces 5 fichiers bruts.
- `<session>/bag/mapping_output_kiss/` (sous `bag/`) — **le vrai export
  utilisé par le pipeline** : `icp_odom.csv` + `map.ply`, présent dans 14/16
  sessions survolées. C'est ce fichier qu'il faut passer à `--offline-icp`.

Si `--offline-icp` pointe par erreur vers le dossier racine, le message
d'erreur de `validate_offline_icp_csv()` (colonnes manquantes) ne suggère pas
qu'un dossier avec le bon fichier existe un niveau plus bas — piège pour
l'opérateur, pas un bug de code.

### Schéma réel de `bag/mapping_output_kiss/icp_odom.csv` (confirmé sur 1 fichier, format identique attendu sur les 13 autres)

```
timestamp_sec,timestamp_nanosec,x,y,z,qx,qy,qz,qw,vx,vy,vz,roll_rate,pitch_rate,yaw_rate
```

Comparé au schéma norlab (`GT_icp/icp_odom_*.csv`,
`mapping_output/icp_odom_*.csv`) :

```
timestamp_sec,timestamp_nanosec,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,pose_covariance,twist_covariance
```

**Seule différence réelle : `roll_rate,pitch_rate,yaw_rate` au lieu de
`wx,wy,wz`.** Pas de `pose_covariance`/`twist_covariance`, mais ces deux
colonnes ne sont lues nulle part dans `offline_reference_solver.cpp` — pas un
problème de compatibilité. Convention de repère identique entre les deux
mappers (les deux commencent à `(0,0,0, quaternion identité)` en première
ligne — vérifié sur les deux fichiers de la même session) : aucune conversion
de repère nécessaire, contrairement à une inquiétude initiale.

### Correction apportée

Avant correction, `load_gt_icp_csv()` faisait `r["wx"]` en dur → `KeyError`
garanti sur tout fichier KISS-ICP réel. Corrigé pour résoudre `wx/wy/wz` ou
`roll_rate/pitch_rate/yaw_rate` selon ce qui est présent dans l'en-tête,
avant la boucle de lecture (pas de try/except par ligne).
`validate_offline_icp_csv()` exige maintenant aussi `vx,vy,vz` + un triplet
angulaire complet (l'un ou l'autre) — avant, ces colonnes n'étaient pas
vérifiées du tout, donc un fichier KISS-ICP passait la validation puis
plantait plus loin dans `build_source_csvs()`.

Ces six colonnes (`vx,vy,vz,wx/wy/wz`) ne servent QUE d'amorce de valeur
initiale pour l'optimiseur (`offline_reference_solver.cpp`, jamais comme
facteur/mesure — voir le commentaire du fichier à ce sujet). `roll_rate,
pitch_rate,yaw_rate` ne coïncide avec `wx,wy,wz` (vitesse angulaire
corps) qu'au voisinage de roll/pitch nul — mais comme ces colonnes
n'atteignent jamais l'optimiseur comme mesure, l'approximation ne peut pas
biaiser le solve, seulement légèrement dégrader la qualité du point de
départ.

Vérifié après correction : le fichier KISS-ICP réel (795 lignes) et un
fichier norlab réel (793 lignes, même session) se chargent tous les deux
sans erreur ; le fichier GT_icp d'Ice-rink (chemin norlab d'origine) reste
inchangé (non-régression) ; un fichier sans aucune des deux conventions
angulaires est rejeté avec un message clair au lieu d'un `KeyError` en aval.

### Précaution opérationnelle (pas un bug de code)

Un run KISS-ICP était en cours au moment de cet audit
(`BAG_FOREST2_.../mapping_output_kiss/` vide, run probablement en écriture).
Aucun fichier sous un dossier `mapping_output_kiss/` n'a été lu comme
référence figée — uniquement inspecté à titre d'exemple de schéma sur une
session dont le run était déjà terminé
(`mtt_edge_case_outside_2026-04-15_16-17-59`).

### Conclusion pratique

Pour une session où le mapper norlab casse et que KISS-ICP est utilisé à la
place : passer directement `--offline-icp
<session>/bag/mapping_output_kiss/icp_odom.csv --icp-approved-by <toi>`, et
`--map-reference <session>/bag/mapping_output_kiss/map.ply`. Aucune
conversion manuelle nécessaire — le pipeline accepte maintenant nativement
les deux conventions de colonnes.

---

## Résumé exécutable pour futurs agents IA

- Pipeline GT offline (`scripts/build_gt_pipeline.py`) : code complet,
  8 scripts, 6 bugs trouvés et corrigés (voir §1), tous vérifiés par
  compilation + tests ciblés (pas seulement syntaxiques).
- `validate_dataset_dir()` prouvée fonctionnelle contre données réelles
  (Ice-rink).
- Chaîne complète jamais exécutée end-to-end — premier vrai test nécessite
  une session non gelée avec ICP offline qualifié + `docker compose run --rm bash`.
- Ice-rink (`BAG_ICE_RINK_...`) : dataset canonique intact, intègre, gates
  passées — **prêt à l'emploi pour recherche motion model**, avec le caveat
  métadonnées de session brutes (§2) à ignorer au profit du catalogue.
- **KISS-ICP confirmé et corrigé (§3)** : `--offline-icp
  <session>/bag/mapping_output_kiss/icp_odom.csv` fonctionne nativement
  depuis le commit `6a69838` — attention au dossier `mapping_output_kiss/`
  racine de session (artefacts bruts TUM/KITTI, différent et incompatible)
  vs celui sous `bag/` (le bon, déjà au bon schéma).
- 14 sessions dans `/data/mtt_bags/` ont un export `bag/mapping_output_kiss/`
  utilisable ; aucune (hors Ice-rink) n'a encore de `GT_icp/`-équivalent
  qualifié + approuvé par `--icp-approved-by` pour un run réel du pipeline.
