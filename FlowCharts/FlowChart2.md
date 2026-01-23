```mermaid
flowchart TD
  S["START<br/>drive"]:::start --> I["Init"]:::proc --> L["Loop"]:::proc

  L --> T{"Timeout?"}:::dec
  T -->|Yes| C["Cleanup<br/>stopDrive(brake)"]:::proc --> E["END"]:::start

  T -->|No| R["Read sensors<br/>distErr, headErr"]:::proc
  R --> H{"stopLatch?"}:::dec

  H -->|Yes| HX{"Exit hold?<br/>abs(distErr) > stopExit"}:::dec
  HX -->|Yes| HR["Exit hold<br/>stopLatch=false"]:::proc --> L
  HX -->|No| HC["Heading hold<br/>headPID -> wCmd"]:::proc --> HO["tankDrive(wCmd, -wCmd)"]:::io
  HO --> HT["holdTimer += dt"]:::proc
  HT --> HD{"holdTimer >= stopSettle?"}:::dec
  HD -->|Yes| C
  HD -->|No| L

  H -->|No| CAP["Dynamic cap"]:::proc --> PID["PID compute<br/>distPID -> v<br/>headPID -> w"]:::proc
  PID --> SC["Scale heading correction"]:::proc --> SL["Slew limit<br/>vCmd, wCmd"]:::proc

  SL --> Z{"Crossed target?"}:::dec
  Z -->|Yes| Z1["crossed=true"]:::proc --> EN
  Z -->|No| EN["Continue"]:::proc

  EN --> EL{"Enter hold?<br/>abs(distErr) < stopEnter<br/>OR crossed"}:::dec
  EL -->|Yes| LCH["Latch hold<br/>stopLatch=true<br/>vCmd=0, wCmd=0"]:::proc --> OUT
  EL -->|No| OUT["Output<br/>left=vCmd+wCmd<br/>right=vCmd-wCmd"]:::proc --> TDV["tankDrive(left,right)"]:::io

  TDV --> ST{"Settled?<br/>small error + low speed<br/>for N ms"}:::dec
  ST -->|Yes| C
  ST -->|No| U["Update + wait(dt)"]:::proc --> L

  classDef start fill:#111827,stroke:#60a5fa,stroke-width:2px,color:#ffffff;
  classDef dec fill:#fef3c7,stroke:#f59e0b,stroke-width:2px,color:#111827;
  classDef proc fill:#e5e7eb,stroke:#6b7280,stroke-width:1.5px,color:#111827;
  classDef io fill:#dcfce7,stroke:#22c55e,stroke-width:2px,color:#111827;
