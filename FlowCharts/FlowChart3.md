```mermaid
flowchart TD
  SH0([STOP-AND-HOLD PHASE]) --> SH1[Read Heading<br/>currHead=headingDeg<br/>headErr=angleDiff(holdHead, currHead)]

  SH1 --> SH2{Exit hold?<br/>|distErr| > stopExit}
  SH2 -- Yes --> SH3[stopLatch = false<br/>Return to normal control] --> SHEND([Return to drive loop])

  SH2 -- No --> SH4[Heading-only correction<br/>w = headPID.update(-headErr, dt)<br/>if |headErr|<1° => w=0, wCmd=0<br/>else wCmd += slew(w - wCmd)]

  SH4 --> SH5[Apply Turn-only Output<br/>tankDrive(wCmd, -wCmd)]
  SH5 --> SH6[Increment Hold Timer<br/>stopHoldMs += dt]

  SH6 --> SH7{stopHoldMs >= stopSettleMs?}
  SH7 -- Yes --> SH8[Break / Exit entire drive loop] --> SHEND
  SH7 -- No --> SH1
