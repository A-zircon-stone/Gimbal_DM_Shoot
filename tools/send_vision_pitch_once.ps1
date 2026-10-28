param([string]$Port='', [switch]$SelfTest)
$ErrorActionPreference='Stop'

function New-VisionPacket([single]$PitchRad) {
    if([single]::IsNaN($PitchRad) -or [single]::IsInfinity($PitchRad)){throw 'Invalid pitch'}
    [byte[]]$packet=New-Object byte[] 31
    $packet[0]=0x53; $packet[1]=0x50; $packet[2]=1
    [BitConverter]::GetBytes($PitchRad).CopyTo($packet,16)
    [int]$crc=0xFFFF
    for($i=0;$i -lt 29;$i++) {
        $crc=$crc -bxor $packet[$i]
        for($j=0;$j -lt 8;$j++) {
            if($crc -band 1){$crc=($crc -shr 1) -bxor 0x8408}else{$crc=$crc -shr 1}
        }
    }
    $packet[29]=[byte]($crc -band 255); $packet[30]=[byte](($crc -shr 8) -band 255)
    return ,$packet
}

if($SelfTest){
    $p=New-VisionPacket (-1.0)
    if($p.Length -ne 31 -or $p[29] -ne 0xD5 -or $p[30] -ne 0x1D){throw 'CRC self-test failed'}
    Write-Output ([BitConverter]::ToString($p))
    Write-Output 'CRC/31-byte packet self-test passed. No serial port opened.'
    exit 0
}

# Only select the firmware's STM32 USB CDC interface, NEVER a DAPLink USART bridge.
if(!$Port){
    $ports=@(Get-CimInstance Win32_SerialPort | Where-Object {$_.PNPDeviceID -match 'VID_0483&PID_5740'})
    if($ports.Count -ne 1){throw 'Connect the C-board USB data cable. Expected exactly one STM32 USB CDC port (0483:5740). DAPLink/USART1 COM is NOT this port.'}
    $Port=$ports[0].DeviceID
}
Write-Host "USB port: $Port. Keep VOFA on the USART1 port; do not open this USB port in another app."
$serial=New-Object System.IO.Ports.SerialPort($Port,115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$serial.ReadTimeout=500; $serial.WriteTimeout=1000
try {
    $serial.Open()
    while($true){
        Write-Host 'On USART1 send vision_enable + CRLF. Support the gimbal, keep clear of the end stops.'
        [void](Read-Host 'Press Enter to read current angle and prepare ONE +2 degree target')
        $serial.DiscardInBuffer()
        $watch=[Diagnostics.Stopwatch]::StartNew(); $row=$null
        while($watch.ElapsedMilliseconds -lt 5000){
            try{$line=$serial.ReadLine().Trim()}catch [TimeoutException]{continue}
            if($line.StartsWith('pitch:')){
                $fields=$line.Substring(6).Split(',')
                $fields=@($fields | ForEach-Object { ($_ -split '=',2)[-1] })
                if($fields.Count -eq 7){$row=$fields; break}
            }
        }
        if(!$row){throw 'No compact pitch telemetry received over USB within 5 seconds. Check firmware/cable/port.'}
        $culture=[Globalization.CultureInfo]::InvariantCulture
        $angle=[double]::Parse($row[0],$culture); $state=[int]$row[6]; $ep=[int]$row[4]
        if($state -ne 1){throw "Pitch loop not enabled/valid (state=$state). Send vision_enable on USART1 first."}
        if([double]::IsNaN($angle) -or [double]::IsInfinity($angle)){throw 'Invalid angle'}
        $target=$angle+2.0
        $expectedEp=$ep-2.0*8192/360
        if($expectedEp -le 3550 -or $expectedEp -ge 6350){throw 'The +2 degree test is too close to an encoder limit. Move to mid-travel first.'}
        [byte[]]$packet=New-VisionPacket ([single]($target*[Math]::PI/180))
        Write-Host ('Current={0:F2} deg, target={1:F2} deg (+2 deg UP), EP={2}' -f $angle,$target,$ep)
        Write-Host ('Packet: '+[BitConverter]::ToString($packet))
        # One binary packet only. CRC, mode=1, fire=0, yaw fields=0. No UART commands.
        $serial.Write($packet,0,$packet.Length)
        Write-Host 'Sent exactly ONE 31-byte packet. Observe USART1 actual/target angle. Stop with pitch_disable.'
        $logDir=Join-Path $PSScriptRoot '../logs/vision-mock'
        New-Item -ItemType Directory -Force $logDir|Out-Null
        ('port={0} actual_deg={1} target_deg={2} hex={3}' -f $Port,$angle,$target,[BitConverter]::ToString($packet)) |
            Set-Content -LiteralPath (Join-Path $logDir ((Get-Date -Format 'yyyyMMdd-HHmmss')+'.txt'))
        break
    }
} finally {if($serial.IsOpen){$serial.Close()}; $serial.Dispose()}
