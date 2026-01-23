```mermaid
flowchart TD
  S["START<br/>turnTo / turnBy"]:::start --> I["Init<br/>turnPID reset<br/>wCmd=0<br/>rateFilt=0"]:::proc --> L["Loop"]:::proc

  L --> T{"Timeout?"}:::dec
  T -->|Yes| C["Cleanup<br/>stopDrive(brake)"]:::proc --> E["END"]:::start

  T -->|No| ER["Compute error"]:::proc
  ER --> RT["Rate (filtered)"]:::proc
  RT --> PID["turnPID -> w"]:::proc
  PID --> SL["Slew -> wCmd"]:::proc
  SL --> OUT["tankDrive(wCmd, -wCmd)"]:::io

  OUT --> ST{"Settled?<br/>abs(err) small<br/>AND abs(rateFilt) small<br/>for N ms"}:::dec
  ST -->|Yes| C
  ST -->|No| U["wait(dt)"]:::proc --> L

  classDef start fill:#111827,stroke:#60a5fa,stroke-width:2px,color:#ffffff;
  classDef dec fill:#fef3c7,stroke:#f59e0b,stroke-width:2px,color:#111827;
  classDef proc fill:#e5e7eb,stroke:#6b7280,stroke-width:1.5px,color:#111827;
  classDef io fill:#dcfce7,stroke:#22c55e,stroke-width:2px,color:#111827;
