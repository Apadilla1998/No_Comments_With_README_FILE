flowchart TD
  U[User / Autonomous Code] --> MC

  subgraph MC[Motion Controller]
    direction TB

    subgraph HL[High-Level Methods]
      direction LR
      d1[drive()] --- t1[turnTo()] --- tb1[turnBy()] --- ac1[autoCorrect()]
      d2[driveAC()] --- t2[turnToAC()] --- tb2[turnByAC()]
      d3[driveCC()]:::muted
    end

    subgraph PID[PID Controllers]
      direction LR
      p1[distPID<br/>(forward)] --- p2[headPID<br/>(heading)] --- p3[turnPID<br/>(turn)]
    end

    HL --> PID
  end

  MC --> OP

  subgraph OP[Output Processing]
    direction LR
    s1[Slew Rate<br/>Limiting] --> s2[Speed<br/>Capping] --> s3[tankDrive(left,right)]
  end

  classDef muted fill:#f6f6f6,stroke:#bbb,color:#444;
