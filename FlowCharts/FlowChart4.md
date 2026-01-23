```mermaid
flowchart TD
  S([autoCorrect(...)]):::start --> EN{Enabled?}:::dec
  EN -->|No| E([Return]):::start
  EN -->|Yes| I["Init<br/>iteration=0<br/>maxIterations=2"]:::proc --> L{{Loop}}:::proc

  L --> ERR["Compute error<br/>dist, dHead, localX, localY"]:::proc

  ERR --> X{Within exit tolerances?}:::dec
  X -->|Yes| E

  X -->|No| LIM{iteration >= maxIterations?}:::dec
  LIM -->|Yes| E

  LIM -->|No| H{Heading needs fix?<br/>abs(dHead) > ENTER_ANG}:::dec
  H -->|Yes| H1["turnTo(targetHead)"]:::proc --> R
  H -->|No| R["Recompute error"]:::proc

  R --> B{Target behind?<br/>localY < -eps}:::dec
  B -->|Yes| BK["drive(-BACKUP_M)"]:::proc --> LAT
  B -->|No| LAT["Lateral check"]:::proc

  LAT --> LX{abs(localX) > ENTER_POS?}:::dec
  LX -->|Yes| SL["turnBy(±90)"]:::proc --> SD["drive(stepX)"]:::proc --> RB["turnTo(targetHead)"]:::proc --> FWD
  LX -->|No| FWD["Forward check"]:::proc

  FWD --> LY{abs(localY) > ENTER_POS?}:::dec
  LY -->|Yes| FD["drive(stepY)"]:::proc --> FIN
  LY -->|No| FIN["Final heading check"]:::proc

  FIN --> INC["iteration++"]:::proc --> L

  classDef start fill:#111827,stroke:#60a5fa,stroke-width:2px,color:#ffffff;
  classDef dec fill:#fef3c7,stroke:#f59e0b,stroke-width:2px,color:#111827;
  classDef proc fill:#e5e7eb,stroke:#6b7280,stroke-width:1.5px,color:#111827;
  classDef io fill:#dcfce7,stroke:#22c55e,stroke-width:2px,color:#111827;
