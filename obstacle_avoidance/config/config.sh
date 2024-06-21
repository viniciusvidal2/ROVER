#!/bin/bash

# Função para validar um endereço IP
validate_ip() {
  local ip="$1"
  if [[ $ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    return 0 # Endereço IP válido
  else
    return 1 # Endereço IP inválido
  fi
}

# Função para validar um dispositivo serial
validate_serial() {
  local serial="$1"
  if [[ $serial =~ ^/dev/tty[A-Z0-9]+$ ]]; then
    return 0 # Dispositivo serial válido
  else
    return 1 # Dispositivo serial inválido
  fi
}

read -p "Você deseja fazer uma simulação SITL? (S/n): " sitl_option

if [[ "$sitl_option" == "S" || "$sitl_option" == "s" ]]; then
  read -p "Informe o endereço IP da Jetson: " fcu_ip
  read -p "Informe o endereço IP do desktop: " gcs_ip

  if ! validate_ip "$fcu_ip" || ! validate_ip "$gcs_ip"; then
    echo "Endereço IP inválido. Certifique-se de inserir um endereço IP no formato xxx.xxx.xxx.xxx."
    exit 1
  fi

  template_path="$(dirname "${BASH_SOURCE[0]}")/../launch/apm_sitl.launch"
else
  read -p "Informe o endereço do dispositivo serial (por exemplo, /dev/ttyACM0): " serial_device
  read -p "Informe o endereço IP para GCS (GCS_IP): " gcs_ip

  if ! validate_serial "$serial_device" || ! validate_ip "$gcs_ip"; then
    echo "Dispositivo serial ou endereço IP inválido. Certifique-se de inserir um dispositivo serial no formato /dev/ttyACM0 e um endereço IP no formato xxx.xxx.xxx.xxx."
    exit 1
  fi

  fcu_ip="$serial_device"
  gcs_ip_default="GCS_IP"
  template_path="$(dirname "${BASH_SOURCE[0]}")/../launch/apm.launch"
fi




output_path="$(dirname "${BASH_SOURCE[0]}")/../launch/apm_mig.launch"

if [ -f "$template_path" ]; then
  cp "$template_path" "$output_path"
  export GCS_IP="$gcs_ip"

  if [[ "$sitl_option" == "S" || "$sitl_option" == "s" ]]; then
    sed -i "s/udp:\/\/FCU_IP:14552@GCS_IP:14552/udp:\/\/$fcu_ip:14552@$gcs_ip:14552/g" "$output_path"
    export FCU_IP="$fcu_ip"
    sed -i "s/udp:\/\/FCU_IP:14550@GCS_IP:14550/udp:\/\/$fcu_ip:14550@$gcs_ip:14550/g" "$output_path"

  else
    sed -i "s#<arg name=\"fcu_url\" default=\"/dev/ttyACM0:57600\" />#<arg name=\"fcu_url\" default=\"$fcu_ip:57600\" />#" "$output_path"
    export FCU_IP="$serial_device"
    sed -i "s/udp:\/\/GCS_IP:14560/udp:\/\/@$gcs_ip:14560/g" "$output_path"

  fi
  
setenv_file="$(dirname "${BASH_SOURCE[0]}")/setenv.sh"

cat > "$setenv_file" << EOL
#!/bin/bash

export FCU_IP="$fcu_ip"
export GCS_IP="$gcs_ip"
EOL

chmod +x "$setenv_file"  



  echo "Arquivo ajustado com sucesso. Verifique $output_path."
else
  echo "Erro: O arquivo $template_path não foi encontrado."
fi
