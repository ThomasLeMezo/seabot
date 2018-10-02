ADC_COEFF=0.015625

B1_hex=`i2cget -y 1 0x39 0x00 w`
B1=$(([##10]$B1_hex))
echo "Battery 1 = $((${B1}*${ADC_COEFF}))"

B1_hex=`i2cget -y 1 0x39 0x02 w`
B1=$(([##10]$B1_hex))
echo "Battery 2 = $((${B1}*${ADC_COEFF}))"

B1_hex=`i2cget -y 1 0x39 0x04 w`
B1=$(([##10]$B1_hex))
echo "Battery 3 = $((${B1}*${ADC_COEFF}))"

B1_hex=`i2cget -y 1 0x39 0x06 w`
B1=$(([##10]$B1_hex))
echo "Battery 4 = $((${B1}*${ADC_COEFF}))"
