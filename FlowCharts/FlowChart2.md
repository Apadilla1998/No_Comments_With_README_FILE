```mermaid
flowchart TD
  A([drive(distM, timeoutMs, maxSpeedPct)]) --> B[Initialization<br/>holdHead = currentHeading<br/>startDist = avgEncDist<br/>targetDist = startDist + distM<br/><br/>distPID.setSetpoint(targetDist)<br/>distPID.reset(startDist)<br/>headPID.setSetpoint(0)<br/>headPID.reset(0)<br/><br/>vCmd=0, wCmd=0<br/>stopLatch=false, crossed=false]

  B --> C[Start Timer<br/>elapsedMs=0, settledMs=0<br/>prevErr = targetDist - startDist]

  C --> D{Timeout?<br/>elapsedMs >= timeoutMs}
  D -- Yes --> Z[Cleanup<br/>stopDrive(brake)] --> END([Return])
  D -- No --> E[Read Sensors<br/>currDist = avgEncDist<br/>currHead = headingDeg<br/>distErr = targetDist - currDist<br/>headErr = angleDiff(holdHead, currHead)]

  E --> F{stopLatch == true?}

  F -- Yes --> SH[Stop-and-Hold Phase<br/>(see separate diagram)] --> D
  F -- No --> G[Dynamic Speed Cap<br/>cap = minCap + 200*|distErr|<br/>cap = min(cap, maxSpeedPct)<br/>distPID.outputLimits = ±cap]

  G --> H[PID Outputs<br/>v = distPID.update(currDist, dt)<br/>w = headPID.update(-headErr, dt)]

  H --> I[Heading Correction Scaling<br/>wScale = min(1.0, |v|/18.0)<br/>wScale = max(wScale, 0.25)<br/>if |v|<12 && |headErr|>2° => wScale=max(wScale, 0.22)<br/>w = w * wScale]

  I --> J[Slew Rate Limiting<br/>dvMax = dvPerSec*dt<br/>dwMax = dwPerSec*dt<br/>vCmd += clamp(v - vCmd, -dvMax, dvMax)<br/>wCmd += clamp(w - wCmd, -dwMax, dwMax)]

  J --> K{Zero-crossing?<br/>(prevErr>0 && distErr<0)<br/>or (prevErr<0 && distErr>0)}
  K -- Yes --> K1[crossed = true]
  K -- No --> K2[no change]
  K1 --> L
  K2 --> L

  L{Enter stopLatch?<br/>!stopLatch && (|distErr|<stopEnter || crossed)}
  L -- Yes --> L1[stopLatch=true<br/>vCmd=0, wCmd=0] --> M
  L -- No --> M

  M[Apply Motor Output<br/>left = vCmd + wCmd<br/>right = vCmd - wCmd<br/>tankDrive(left, right)] --> N[Settling Check<br/>if |distErr|<0.02 && |vCmd|<6<br/>  settledMs += dt<br/>else settledMs=0]

  N --> O{settledMs >= 40ms?}
  O -- Yes --> Z
  O -- No --> P[Update State<br/>prevErr=distErr<br/>elapsedMs += dt<br/>wait(dt)] --> D
