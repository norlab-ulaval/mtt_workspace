# ICP Mapper Configuration Reference

> **Source**: libpointmatcher v1.4.4 / norlab_icp_mapper_ros / norlab_robot
> **Config files**: `src/external/norlab_robot/config/mapping/_config.yaml` (default), `_config_hesai_imu_replay.yaml` (offline IMU replay)

---

## Structure générale d'un fichier YAML

```yaml
input:                          # Filtres sur le nuage entrant (lecture + map level)
  - <DataPointsFilterName>:
      <param>: <value>

icp:
  matcher:                      # Un seul matcher
    <MatcherName>:
      <param>: <value>

  outlierFilters:               # 0 ou plusieurs, en séquence
    - <OutlierFilterName>:
        <param>: <value>

  errorMinimizer:               # Un seul minimiseur
    <ErrorMinimizerName>:
      <param>: <value>

  transformationCheckers:       # 0 ou plusieurs, condition OU
    - <TransformationCheckerName>:
        <param>: <value>

  inspector:                    # Optionnel, debug
    <InspectorName>:
      <param>: <value>

  logger:                       # Optionnel
    <LoggerName>:
      <param>: <value>

post:                           # Filtres sur le nuage après ICP (typiquement normales finales)
  - <DataPointsFilterName>:
      <param>: <value>

mapper:                         # Paramètres haute niveau du noeud mapper
  updateCondition:
    type: distance
    value: 0.10                 # mètres entre chaque ICP
  sensorMaxRange: 80.0          # portée max du capteur (borne la map)
  mapperModule:
    - PointDistanceMapperModule:
        minDistNewPoint: 0.05   # résolution de la map (m)
```

---

## 1. DataPointsFilter — Filtres de nuage

### 1.1 BoundingBoxDataPointsFilter

Coupe les points dans (ou hors) une boîte alignée sur les axes.

```yaml
- BoundingBoxDataPointsFilter:
    xMin: -0.90
    xMax:  0.70
    yMin: -0.40
    yMax:  0.40
    zMin: -0.10
    zMax:  0.80
    removeInside: 1   # 1 = supprime les points DANS la boîte, 0 = supprime les points HORS boîte
```

**Rôle**: Self-removal du châssis tracteur, cage LiDAR, housing, remorque.
Les bboxes sont exprimées dans `filtering_frame` (généralement `base_link`).
Ne pas mettre de filtres de descripteurs (normales, observationDirection) ici — ils resteraient dans la frame `base_link` après le transform-retour et rendraient ICP incohérent.

---

### 1.2 VoxelGridDataPointsFilter

Sous-échantillonnage uniforme par grille de voxels.

```yaml
- VoxelGridDataPointsFilter:
    vSizeX: 0.10        # taille du voxel en x (m)
    vSizeY: 0.10        # taille du voxel en y (m)
    vSizeZ: 0.10        # taille du voxel en z (m)
    useCentroid: 1      # 1 = centroïde du voxel, 0 = centre géométrique
    averageExistingDescriptors: 1  # moyenner les descripteurs existants
```

**Rôle**: C'est **le filtre obligatoire** pour borner le nombre de points.
- 0.10 m → Hesai XT-32 (~30K pts/scan → ~15K après voxel)
- 0.12 m → offline (plus précis, moins de points)
- 0.15 m → live (léger)

---

### 1.3 SurfaceNormalDataPointsFilter

Calcule les normales de surface par eigendecomposition de la covariance des k-voisins.

```yaml
- SurfaceNormalDataPointsFilter:
    knn: 12             # nombre de voisins pour le calcul
    maxDist: inf        # distance max de recherche
    epsilon: 0          # approximation knn
    keepNormals: 1      # ajouter les normales comme descripteur
    keepDensities: 0    # ajouter la densité locale
    keepEigenValues: 0  # ajouter les valeurs propres
    keepEigenVectors: 0 # ajouter les vecteurs propres
    keepMeanDist: 0     # ajouter la distance moyenne au voisin
    sortEigen: 0        # trier valeurs propres (croissant)
    smoothNormals: 0    # lisser les normales avec les voisins
```

**Rôle**: Nécessaire pour `PointToPlaneErrorMinimizer` et `SurfaceNormalOutlierFilter`.
Placé dans `input:` (pré-ICP) ou `post:` (post-ICP, normales finales de la map).

---

### 1.4 MaxDistDataPointsFilter

Coupe les points au-delà d'une distance.

```yaml
- MaxDistDataPointsFilter:
    dim: -1             # -1 = rayon, 0=x, 1=y, 2=z
    maxDist: 100.0      # distance max (m)
```

---

### 1.5 MinDistDataPointsFilter

Coupe les points en dessous d'une distance.

```yaml
- MinDistDataPointsFilter:
    dim: -1             # -1 = rayon, 0=x, 1=y, 2=z
    minDist: 1.0        # distance min (m)
```

---

### 1.6 RandomSamplingDataPointsFilter

Sous-échantillonnage aléatoire.

```yaml
- RandomSamplingDataPointsFilter:
    prob: 0.5           # probabilité de garder chaque point
    randomSamplingMethod: 0  # 0 = RNG direct, 1 = uniforme
    seed: -1            # -1 = pas de seed
```

---

### 1.7 MaxPointCountDataPointsFilter

Plafonne le nombre de points.

```yaml
- MaxPointCountDataPointsFilter:
    maxCount: 10000     # nombre max de points à garder
    seed: 1             # seed du générateur
```

---

### 1.8 NormalSpaceDataPointsFilter

Échantillonnage qui préserve la distribution des normales.

```yaml
- NormalSpaceDataPointsFilter:
    nbSample: 5000      # nombre de points à sélectionner
    seed: 1             # seed
    epsilon: 0.098      # pas de discrétisation (PI/32 par défaut)
```

**Rôle**: Utile dans les environnements peu structurés (corridors longs) où l'ICP a tendance à glisser.

---

### 1.9 ObservationDirectionDataPointsFilter

Ajoute la direction d'observation depuis le capteur comme descripteur.

```yaml
- ObservationDirectionDataPointsFilter:
    x: 0.0              # position x du capteur
    y: 0.0              # position y du capteur
    z: 0.0              # position z du capteur
```

**Rôle**: Nécessaire pour `ShadowDataPointsFilter` et `IncidenceAngleDataPointsFilter`.
⚠️ Problème connu : sur un nuage fusionné (multi-capteurs), la direction d'observation est fausse car chaque point vient d'une origine différente mais le descripteur n'en connaît qu'une.

---

### 1.10 ShadowDataPointsFilter

Supprime les points dont la normale pointe loin du capteur (ombres).

```yaml
- ShadowDataPointsFilter:
    eps: 0.1            # angle (rad) de tolérance
```

**Rôle**: Nettoyage des bords de discontinuité. Nécessite normales + observationDirections.

---

### 1.11 SamplingSurfaceNormalDataPointsFilter

Sous-échantillonnage + normales par surfels.

```yaml
- SamplingSurfaceNormalDataPointsFilter:
    ratio: 0.5          # ratio de points à garder
    knn: 7              # voisins pour normale, aussi seuil de split
    samplingMethod: 0   # 0 = aléatoire, 1 = bin (1/knn points)
    maxBoxDim: inf      # taille max de boîte
    keepNormals: 1      # garder les normales
    keepDensities: 0    # garder les densités
    keepEigenValues: 0  # garder les valeurs propres
    keepEigenVectors: 0 # garder les vecteurs propres
```

---

### 1.12 OctreeGridDataPointsFilter

Sous-échantillonnage par octree (alternative à VoxelGrid).

```yaml
- OctreeGridDataPointsFilter:
    buildParallel: 1         # construction parallèle
    maxPointByNode: 1        # points max par feuille
    maxSizeByNode: 0         # taille max de boîte (0 = désactivé)
    samplingMethod: 2        # 0=premier, 1=aléatoire, 2=centroïde, 3=médoïde
```

---

### 1.13 DistanceLimitDataPointsFilter

Version plus flexible de MaxDist/MinDist avec toggle inside/outside.

```yaml
- DistanceLimitDataPointsFilter:
    dim: -1             # -1=rayon, 0=x, 1=y, 2=z
    dist: 1.0           # limite
    removeInside: 1     # 1 = supprime AVANT la limite, 0 = supprime APRÈS
```

---

### 1.14 MaxDensityDataPointsFilter

Sous-échantillonnage pour atteindre une densité cible.

```yaml
- MaxDensityDataPointsFilter:
    maxDensity: 10.0    # points par m³
```

---

### 1.15 CovarianceSamplingDataPointsFilter

Sélectionne les points qui maximisent l'information pour l'ICP.

```yaml
- CovarianceSamplingDataPointsFilter:
    nbSample: 5000      # nombre de points
    torqueNorm: 1       # 0=sans normalisation, 1=moyenne, 2=maximum
```

---

### 1.16 Autres DataPointsFilter (utilisation occasionnelle)

| Filtre | Paramètres | Rôle |
|---|---|---|
| `IdentityDataPointsFilter` | — | Passe-trappe (ne fait rien) |
| `RemoveNaNDataPointsFilter` | — | Supprime les points NaN |
| `AngleLimitDataPointsFilter` | `phiMin, phiMax, thetaMin, thetaMax, removeInside` | Coupe par angle sphérique |
| `MaxQuantileOnAxisDataPointsFilter` | `dim, ratio, removeBeyond` | Coupe par quantile sur un axe |
| `FixStepSamplingDataPointsFilter` | `startStep, endStep, stepMult` | Sous-échantillonnage à pas régulier variable |
| `CutAtDescriptorThresholdDataPointsFilter` | `descName, threshold, useLargerThan` | Coupe par valeur de descripteur |
| `SimpleSensorNoiseDataPointsFilter` | `sensorType, gain` | Ajoute un descripteur de bruit capteur |
| `RemoveSensorBiasDataPointsFilter` | `sensorType, angleThreshold` | Corrige le biais systématique LiDAR |
| `OrientNormalsDataPointsFilter` | `towardCenter` | Oriente les normales vers/loin du capteur |
| `IncidenceAngleDataPointsFilter` | — | Calcule l'angle d'incidence (normale × direction) |
| `ElipsoidsDataPointsFilter` | `ratio, knn, keep*` | Surfel ellipsoïdal (recherche) |
| `GestaltDataPointsFilter` | `ratio, radius, knn, keep*` | Descripteurs Gestalt (shape context) |
| `SphericalityDataPointsFilter` | `keepUnstructureness, keepStructureness` | Métriques de forme à partir des valeurs propres |
| `SaliencyDataPointsFilter` | `k, sigma, keep*` | Tensor Voting (surface/jonction/plaque) |
| `SpectralDecompositionDataPointsFilter` | `k, sigma, radius, itMax, keep*` | Décomposition spectrale itérative |
| `AddDescriptorDataPointsFilter` | `descriptorName, descriptorDimension, descriptorValues` | Ajoute un descripteur constant |

---

## 2. Matcher — Appariement des points

### 2.1 KDTreeMatcher

**Le seul utile en pratique.** Kd-tree via libnabo.

```yaml
matcher:
  KDTreeMatcher:
    knn: 7              # nombre de plus proches voisins dans la map
    maxDist: 1.0        # rayon de recherche max (m)
    epsilon: 0          # approximation (0 = exact, 1.0 = 100% plus rapide)
    searchType: 1       # 0 = brute force, 1 = kd-tree linear heap (knn < 30), 2 = kd-tree tree heap (knn >= 30)
```

**Recommandations**:
- `knn=7` : standard 3D
- `maxDist` : compromis critique — trop petit = rien trouvé en virage, trop grand = faux matches en corridor
  - 1.0 m : indoor/outdoor modéré (défaut)
  - 4.0 m : offline IMU (tolère les grands écarts)
  - 0.7 m : trop restrictif en virage

### 2.2 KDTreeVarDistMatcher

Distance de recherche variable par point (lue depuis un descripteur).

```yaml
matcher:
  KDTreeVarDistMatcher:
    knn: 1
    maxDistField: "maxSearchDist"   # descripteur contenant la distance par point
    epsilon: 0
    searchType: 1
```

### 2.3 NullMatcher

Ne matche rien. Pour test seulement.

---

## 3. OutlierFilter — Rejet des faux appariements

### 3.1 TrimmedDistOutlierFilter

**Le plus simple et robuste.** Garde les meilleurs `ratio`% matches.

```yaml
- TrimmedDistOutlierFilter:
    ratio: 0.75         # garde les 75% meilleurs matches (distance la plus petite)
```

Basé sur Chetverikov 2002 (Trimmed ICP).

### 3.2 RobustOutlierFilter

**M-estimateur** — pondère les matches au lieu de les couper. Plus stable que TrimmedDist.

```yaml
- RobustOutlierFilter:
    robustFct: cauchy           # cauchy, welsch, sc, gm, tukey, huber, L1, Student
    scaleEstimator: mad          # mad, none, berg
    distanceType: point2plane    # point2point ou point2plane
    tuning: 1.0                  # paramètre d'ajustement
    approximation: inf           # seuil au-delà duquel poids=0 (accélération)
```

**Fonctions robustes** :
- `cauchy` : douce, bon compromis (recommandé)
- `huber` : plus dure (passe de L2 à L1)
- `tukey` : rejette complètement les outliers forts
- `welsch` : très douce
- `gm` : Geman-McClure
- `L1` : valeur absolue
- `sc` : Switchable-Constraint

### 3.3 SurfaceNormalOutlierFilter

Rejette les matches où les normales divergent.

```yaml
- SurfaceNormalOutlierFilter:
    maxAngle: 1.0       # angle max entre normales (rad) — 1.0 rad ≈ 57°
```

**Rôle**: Empêche les faux matches mur ↔ sol. Nécessite normales sur les deux nuages.

### 3.4 MaxDistOutlierFilter

Seuil fixe.

```yaml
- MaxDistOutlierFilter:
    maxDist: 1.0        # distance max
```

### 3.5 MedianDistOutlierFilter

Seuil relatif basé sur la médiane.

```yaml
- MedianDistOutlierFilter:
    factor: 3.0         # rejette les matches > factor × médiane
```

### 3.6 VarTrimmedDistOutlierFilter

Version automatique de TrimmedDist (optimise le ratio).

```yaml
- VarTrimmedDistOutlierFilter:
    minRatio: 0.05
    maxRatio: 0.99
    lambda: 2.35
```

### 3.7 GenericDescriptorOutlierFilter

Pondération par descripteur.

```yaml
- GenericDescriptorOutlierFilter:
    source: reference       # reference ou reading
    descName: "none"        # nom du descripteur
    useSoftThreshold: 0     # 0 = binaire, 1 = poids = valeur
    useLargerThan: 1        # sens du seuil
    threshold: 0.1
```

### 3.8 NullOutlierFilter

Passe-trappe.

```yaml
- NullOutlierFilter:
```

---

## 4. ErrorMinimizer — Minimisation de l'erreur

### 4.1 IdentityErrorMinimizer

**Ne corrige rien.** Retourne la transformation du prior (odométrie).

```yaml
errorMinimizer:
  IdentityErrorMinimizer:
```

**Rôle**: **Diagnostic uniquement** — vérifier si l'odométrie seule produit une carte cohérente.

### 4.2 PointToPlaneErrorMinimizer

**Le standard pour l'ICP 3D.** Distance point→plan tangent.

```yaml
errorMinimizer:
  PointToPlaneErrorMinimizer:
    force4DOF: 1        # 1 = optimise yaw + x,y,z seulement
                        # 0 = 6DOF complet (roll, pitch, yaw, x, y, z)
    force2D: 0          # 1 = force la minimisation 2D (plan XY)
```

**force4DOF=1** (recommandé terrain plat à vallonné) :
- Optimise : yaw, x, y, z
- Laisse : roll, pitch = prior (odom)
- Le TF base_footprint → capteur est statique (URDF). Sans capteur pitch/roll temps réel, l'ICP ne peut pas corriger la distorsion due au pitch. La translation Z compense partiellement.

**force4DOF=0** (6DOF complet) :
- Optimise : roll, pitch, yaw, x, y, z
- Utile sur terrain escarpé (>10%) avec un bon prior IMU pour le pitch/roll

### 4.3 PointToPlaneWithCovErrorMinimizer

PointToPlane + matrice de covariance du résultat.

```yaml
errorMinimizer:
  PointToPlaneWithCovErrorMinimizer:
    force4DOF: 1
    sensorStdDev: 0.01  # écart-type capteur (m)
```

**Rôle**: Plus robuste dans les environnements à faible structure (corridor, champ). La covariance propage l'incertitude des normales dans le solveur. Plus coûteux.

### 4.4 PointToPointErrorMinimizer

Distance point→point (plus simple, moins bon).

```yaml
errorMinimizer:
  PointToPointErrorMinimizer:
```

### 4.5 PointToPointWithCovErrorMinimizer

PointToPoint + covariance.

```yaml
errorMinimizer:
  PointToPointWithCovErrorMinimizer:
    sensorStdDev: 0.01
```

### 4.6 PointToPointSimilarityErrorMinimizer

PointToPoint avec scale (rotation + translation + échelle uniforme).

```yaml
errorMinimizer:
  PointToPointSimilarityErrorMinimizer:
```

---

## 5. TransformationChecker — Conditions d'arrêt

Les checkers sont en condition **OU** (le premier qui déclenche arrête l'ICP).

### 5.1 DifferentialTransformationChecker

Arrêt par convergence.

```yaml
- DifferentialTransformationChecker:
    minDiffRotErr: 0.001     # seuil rotation (rad) entre itérations
    minDiffTransErr: 0.01    # seuil translation (m) entre itérations
    smoothLength: 4          # fenêtre de lissage (moyenne glissante sur N itérations)
```

**Rôle**: Critère principal. La convergence est déclarée quand la MOYENNE GLISSANTE des 4 dernières itérations passe sous le seuil. Ce n'est PAS 4 itérations consécutives — c'est une fenêtre qui lisse les oscillations.

### 5.2 CounterTransformationChecker

Arrêt par nombre max d'itérations.

```yaml
- CounterTransformationChecker:
    maxIterationCount: 50   # nombre max d'itérations
```

**Rôle**: Filet de sécurité. 50 pour temps réel, 100 pour offline.

### 5.3 BoundTransformationChecker

Arrêt par **exception** (convergence error) si la correction dépasse les bornes.

```yaml
- BoundTransformationChecker:
    maxRotationNorm: 0.80      # angle max (rad) — 0.80 rad = 45.8°
    maxTranslationNorm: 3.0    # translation max (m)
```

**Rôle**: Protection contre la divergence. Avec force4DOF=1, seul le yaw est optimisé : `maxRotationNorm` borne la correction d'orientation.
- 0.80 rad = 45.8° — filet de sécurité
- Absent dans la config offline (laisser ICP converger librement)

⚠️ `maxRotationNorm` utilise `Eigen::Quaternion::angularDistance()` — pour une rotation pure en yaw, c'est équivalent à la norme du vecteur de Rodrigues.

---

## 6. Inspector & Logger — Debug

### 6.1 NullInspector

Rien (production).

```yaml
inspector:
  NullInspector:
```

### 6.2 VTKFileInspector

Dump chaque itération ICP → VTK (visualisable dans ParaView).

```yaml
inspector:
  VTKFileInspector:
    baseFileName: "point-matcher-output"
    dumpReading: 1         # dump nuage lecture
    dumpReference: 1       # dump nuage référence
    dumpDataLinks: 0       # dump liens appariés
    writeBinary: 0         # VTK binaire
    precision: 7           # précision d'écriture
```

### 6.3 PerformanceInspector

Stats de performance.

```yaml
inspector:
  PerformanceInspector:
    baseFileName: ""       # fichier de sortie (vide = stdout)
    dumpPerfOnExit: 0      # dump perf à la sortie
    dumpStats: 0           # dump stats
```

### 6.4 Logger

```yaml
logger:
  NullLogger:              # Rien (production)
  # ou
  FileLogger:
    infoFileName: "icp.log"
    warningFileName: "icp_warn.log"
    displayLocation: 0     # afficher la ligne de code source
```

---

## 7. Mapper — Paramètres haute niveau

Ces paramètres sont dans le YAML `input:`/`icp:`/`mapper:` mais lus par le noeud mapper.

```yaml
mapper:
  updateCondition:
    type: distance            # condition : distance
    value: 0.10               # déclenche une mise à jour ICP tous les 10 cm

  sensorMaxRange: 80.0        # portée max du capteur (borne la croissance de la map)

  mapperModule:
    - PointDistanceMapperModule:
        minDistNewPoint: 0.05 # distance min entre nouveaux points de la map (résolution)
```

---

## 8. Paramètres ROS 2 du noeud mapper (norlab_icp_mapper_ros)

Ces paramètres sont passés via le launch file, pas dans le YAML ICP.

### 8.1 Quality Gate (filtre les corrections ICP aberrantes)

| Paramètre | Défaut | Rôle |
|---|---|---|
| `minInputPoints` | 100 | Rejette les scans avec trop peu de points |
| `maxTranslationCorrection` | 2.0 m | Correction ICP translation max |
| `maxRotationCorrectionDeg` | 30.0° | Correction ICP rotation max |
| `maxVelocityMs` | 20.0 m/s | Vélocité max du robot |
| `maxYawRateDegS` | 90.0°/s | Taux de lacet max |
| `maxPoseStepM` | 2.0 m | Pas de pose max entre scans acceptés |
| `maxZJumpM` | 0.75 m | Saut en Z max (replay: 2.0 m car recalibration capteur) |
| `maxRegistrationTimeMs` | 5000 ms | Temps ICP max |
| `recoveryAfterRejections` | 10 | Force l'acceptation après N rejets consécutifs (0 = désactivé) |
| `enableConvergenceErrorDump` | false | Sauve map/trajectoire sur convergence error |

### 8.2 Overlap Gates (contrôle insertion map et acceptation pose)

| Paramètre | Défaut | Rôle |
|---|---|---|
| `minPoseOverlapNearRatio` | 0.25 | Rejette la pose si le recouvrement proche est < ce ratio |
| `minPoseOverlapLooseRatio` | 0.45 | Rejette la pose si le recouvrement large est < ce ratio |
| `minMapOverlapNearRatio` | 0.30 | Saute l'insertion map si recouvrement proche < ce ratio |
| `minMapOverlapLooseRatio` | 0.50 | Saute l'insertion map si recouvrement large < ce ratio |
| `maxMapUpdateTranslationCorrectionM` | 1.50 m | Saute l'insertion map si correction ICP > ce seuil |
| `maxMapUpdateRotationCorrectionDeg` | 12.0° | Saute l'insertion map si correction ICP > ce seuil |

**Attention** : En replay, des seuils trop hauts peuvent bloquer l'insertion de la map dans les zones nouvelles (deadlock). Les valeurs en replay sont volontairement basses (0.10/0.20).

### 8.3 Map Trimming

| Paramètre | Défaut Live | Défaut Replay | Rôle |
|---|---|---|---|
| `enableMapTrimming` | true | **false** | Tronque la map pour borner le coût ICP |
| `mapTrimIntervalScans` | 10 | — | Vérification toutes les N acceptations |
| `mapTrimRadiusM` | 40.0 | 80.0 | Rayon de map conservé autour du robot |
| `maxMapPointsBeforeTrim` | 120000 | 2000000 | Points max avant déclenchement du trim |

**Attention replay** : `enableMapTrimming=false` est FATAL si `sensorMaxRange > trimRadius` — après un trim, les points lointains ne matchent plus, l'overlap gate bloque tout, la map gèle.

### 8.4 Déterministic Map Update

| Paramètre | Défaut | Rôle |
|---|---|---|
| `deterministicMapUpdateDistanceM` | 0.10 m | Espacement XY pour insertion déterministe |
| `deterministicMapUpdateYawDeg` | 3.0° | Espacement en lacet |
| `deterministicMapMinDistNewPoint` | 0.05 m | Espacement min entre nouveaux points |

### 8.5 Global Output Map

| Paramètre | Défaut | Rôle |
|---|---|---|
| `enableGlobalOutputMap` | false | Garde une map non tronquée en parallèle |
| `globalOutputMapMinDistNewPoint` | 0.05 m | Résolution de la map globale |

### 8.6 Deskew

| Paramètre | Défaut | Rôle |
|---|---|---|
| `deskew` | true | Correction du skew du LiDAR (mouvement pendant le scan) |
| `expectedUniqueDeskewingTFNumber` | 4000 | Réservations dans le cache TF |
| `deskewingRoundToNanoSecs` | 50000 | Bin des timestamps (résolution) |
| `deskewFixedFrame` | odom | Frame fixe pour l'interpolation TF |
| `deskewTimeMode` | absolute_ns | Mode de temps |
| `deskewTimeField` | time | Champ de temps dans DataPoints |

### 8.7 Autres ROS params

| Paramètre | Défaut | Rôle |
|---|---|---|
| `mapFrame` | map | Frame de la map publiée |
| `odomFrame` | odom | Frame odométrie |
| `robotFrame` | base_link | Frame du robot |
| `filteringFrame` | base_link | Frame pour les bboxes de filtrage |
| `mapPublishRate` | 1.0 Hz | Taux de publication de la map |
| `mapTfPublishRate` | 50.0 Hz | Taux de republication du TF map |
| `is3D` | true | Mode 3D |
| `isMapping` | true | Mode cartographie |
| `isOnline` | true | Live (true) ou bag replay (false) |
| `saveMapCellsOnHardDrive` | false | Sauvegarde disque des cellules |
| `inputQosReliable` | false | QoS fiable pour l'abonnement points |
| `tfLookupTimeoutMs` | 200 | Timeout lookup TF (ms) |
| `compressionVoxelSize` | 0.5 | Voxel size pour map publiée (0 = désactivé) |

---

## 9. Différences clés entre les deux configs

| Paramètre | `_config.yaml` (défaut) | `_config_hesai_imu_replay.yaml` |
|---|---|---|
| **KDTreeMatcher.maxDist** | 1.0 m | 4.0 m |
| **VoxelGrid** | 0.10 m | 0.12 m |
| **OutlierFilters** | TrimmedDist 0.75 | SurfaceNormal 1.0 rad → Cauchy M-est → TrimmedDist 0.85 |
| **ErrorMinimizer** | PointToPlane, force4DOF=1 | PointToPlaneWithCov, force4DOF=1, sensorStdDev=0.01 |
| **CounterTransformationChecker** | 50 iters | 100 iters |
| **BoundTransformationChecker** | 0.80 rad / 3.0 m | **Absent** |
| **PointDistanceMapperModule** | 0.05 m | 0.08 m |
| **Normales dans input** | Non | Oui (pour SurfaceNormalOutlierFilter) |

---

## 10. Chaîne ICP type (résumé visuel)

```
Nuage entrant (30K pts)
  → BoundingBoxDataPointsFilter × 4 (self-removal)
  → VoxelGridDataPointsFilter (0.10 m → 15K pts)
  → [optionnel] SurfaceNormalDataPointsFilter (normales)
  ──────────────────────────────────────────────────
  KDTreeMatcher (maxDist=1.0, knn=7)
    → appariement points lecture ↔ map
  OutlierFilters
    → [optionnel] SurfaceNormalOutlierFilter
    → RobustOutlierFilter / TrimmedDistOutlierFilter
  ErrorMinimizer
    → PointToPlane(WithCov)
  TransformationCheckers
    → DifferentialTransformationChecker (convergence)
    → CounterTransformationChecker (50 iters max)
    → BoundTransformationChecker (protection divergence)
  ──────────────────────────────────────────────────
  Pose corrigée → insertion map (si overlap OK)
```

---

## 11. Liens utiles

- Documentation libpointmatcher : https://libpointmatcher.readthedocs.io/en/latest/
- Code source : `src/external/libpointmatcher/pointmatcher/`
- Configs mapping : `src/external/norlab_robot/config/mapping/`
- Noeud mapper : `src/external/norlab_icp_mapper_ros/src/mapper_node.cpp`
- Launch : `src/external/norlab_robot/launch/`
