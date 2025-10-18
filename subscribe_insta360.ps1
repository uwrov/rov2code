$id_line = usbip list -r 172.25.250.1 | FINDSTR 2e1a
$split_id_line = $id_line -split " "
$id = $split_id_line[3]
echo "Attempting to subscribe busid $id"
usbip.exe attach -r 172.25.250.1 -b $id