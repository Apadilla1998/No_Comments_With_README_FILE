flowchart TD
  A([autoCorrect(targetX, targetY, targetHead, timeoutMs, maxSpeed)]) --> B{autoCorrectEnabled_?}
  B -- No --> END([Return / no-op])
  B -- Yes --> C[iteration=0<br/>maxIterations=2] --> LOOP

  LOOP[Start Correction Loop] --> D[Compute Error<br/>dx = targetX - pose.x<br/>dy = targetY - pose.y<br/>dist = sqrt(dx^2 + dy^2)<br/>currHead = headingDeg<br/>dHead = angleDiff(targetHead, currHead)]

  D --> E{Exit?<br/>dist < EXIT_DIST && |dHead| < EXIT_ANG}
  E -- Yes --> END
  E -- No --> F{iteration >= maxIterations?}
  F -- Yes --> END
  F -- No --> G{Fix heading first?<br/>|dHead| > ENTER_ANG}

  G -- Yes --> G1[turnTo(targetHead, timeoutMs/3)] --> H
  G -- No --> H[Continue]

  H --> I[Recalculate dist after heading fix<br/>dx, dy, dist]

  I --> J[Robot-frame check (behind?)<br/>localY = dx*sin(head) + dy*cos(head)]
  J --> K{localY < -0.02?}
  K -- Yes --> K1[drive(-BACKUP_M, timeoutMs/4, maxSpeed)] --> L
  K -- No --> L

  L --> M[Compute lateral error<br/>localX = dx*cos(head) - dy*sin(head)]
  M --> N{|localX| > ENTER_POS?}

  N -- Yes --> N1[slideAngle = (localX>0) ? 90 : -90<br/>turnBy(slideAngle, timeoutMs/4)] --> N2[slideD = min(|localX|, MAX_STEP_M)<br/>drive(slideD, timeoutMs/4, maxSpeed)] --> N3[turnTo(targetHead, timeoutMs/4)] --> O
  N -- No --> O[Skip lateral fix]

  O --> P[Forward error (recalc dx,dy)<br/>localY = dx*sin(head) + dy*cos(head)]
  P --> Q{|localY| > ENTER_POS?}
  Q -- Yes --> Q1[fwdD = clamp(localY, -MAX_STEP, MAX_STEP)<br/>drive(fwdD, timeoutMs/4, maxSpeed)] --> R
  Q -- No --> R

  R --> S[Final heading check<br/>dHead = angleDiff(targetHead, headingDeg)]
  S --> T{|dHead| > EXIT_ANG?}
  T -- Yes --> T1[turnTo(targetHead, timeoutMs/4)] --> U
  T -- No --> U[Continue]

  U --> V[iteration++]
  V --> LOOP
