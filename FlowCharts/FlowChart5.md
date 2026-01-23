```mermaid
flowchart TB
  U["User / Autonomous Code"]:::io --> MC

  subgraph MC["Motion Controller"]
    direction TB

    subgraph HL["High-Level Methods"]
      direction LR
      d["drive()"]:::proc --> tt["turnTo()"]:::proc --> tb["turnBy()"]:::proc --> ac["autoCorrect()"]:::proc
      dac["driveAC()"]:::proc --> tta["turnToAC()"]:::proc --> tba["turnByAC()"]:::proc
      dcc["driveCC()"]:::proc
    end

    subgraph PID["PID Controllers"]
      direction LR
      dp["distPID"]:::proc --> hp["headPID"]:::proc --> tp["turnPID"]:::proc
    end

    HL --> PID
  end

  MC --> OP

  subgraph OP["Output Processing"]
    direction LR
    slew["Slew Limit"]:::proc --> cap["Speed Cap"]:::proc --> out["tankDrive(left,right)"]:::io
  end

  classDef start fill:#111827,stroke:#60a5fa,stroke-width:2px,color:#ffffff;
  classDef dec fill:#fef3c7,stroke:#f59e0b,stroke-width:2px,color:#111827;
  classDef proc fill:#e5e7eb,stroke:#6b7280,stroke-width:1.5px,color:#111827;
  classDef io fill:#dcfce7,stroke:#22c55e,stroke-width:2px,color:#111827;
