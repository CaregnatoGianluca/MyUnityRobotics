' -----------------------------------------------------
' Mitsubishi iQ Works PURE TCP SOCKET ARCHITECTURE
' Based on RobotArmController_GPT Pipeline
' File: TCPSERVER.prg
' -----------------------------------------------------


' Override OPT11 default settings to become a Client connecting to Unity
C_Com(5)="ETH:127.0.0.1,12345"

Def Float FX, FY, FZ
Def Float PX, PY, PZ
Def Pos PTarg
Def Pos PHome

' Save the initial starting position as Home
PHome = P_Curr

' Open the TCP Socket via COM5 (which is mapped to OPT11)
Open "COM5:" As #1

*LOOP
  ' Tell Unity we are ready
  Print #1, "ROBOT WAITING FOR DATA..."
  
  ' Wait for incoming payload from Unity
  Input #1, FX, FY, FZ
  
  ' Tell Unity what we successfully parsed
  Print #1, "PARSED DATA: ", FX, FY, FZ
  
  ' Divide by 100 to safely restore the decimals (bypasses Locale comma/dot bug)
  PX = FX / 100.0
  PY = FY / 100.0
  PZ = FZ / 100.0
  
  ' Copy structural flags
  PTarg = P_Curr
  
  ' Apply coordinates
  PTarg.X = PX
  PTarg.Y = PY
  PTarg.Z = PZ
  PTarg.A = 180.0
  PTarg.B = 0.0
  PTarg.C = 0.0
  
  ' Execute Pick Sequence
  PTarg.Z = PTarg.Z + 100.0
  Mov PTarg
  
  PTarg.Z = PTarg.Z - 100.0
  Mvs PTarg
  Dly 0.5
  HClose 1
  Dly 0.5
  
  PTarg.Z = PTarg.Z + 100.0
  Mvs PTarg
  
  ' Return to Home Position
  Mov PHome
  
  ' Acknowledge completion back to Unity
  Print #1, "MOVE COMPLETED"
  
GoTo *LOOP
End
