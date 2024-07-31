#!/bin/bash

# Check if the first command-line argument is provided
if [ -n "$1" ]; then
    ota_host=$1
# Check if the environment variable is set
elif [ -n "$OTA_HOST" ]; then
    ota_host=$OTA_HOST
else
    echo "Error: No OTA host provided, please provide the OTA host as the first argument or set the OTA_HOST environment variable."
    exit 1
fi

ota_api_port=18888

set -e

# Make the curl request and store the result in a variable
response=$(curl -s -m 5 http://$ota_host:$ota_api_port/v1/markers)

# Parse the clients array from the response
clients=$(echo $response | jq '.clients')


# Check if the clients array is empty
if [[ $clients == "[]" ]]; then
  echo "Error: No clients found."
  exit 1
fi

# Parse all addresses from the clients array and store them in a list
addresses=$(echo $response | jq -r '.clients[].address')

# Convert the addresses into an array
# shellcheck disable=SC2206
address_list=($addresses)

# Loop through the list and print each address
set +e
for address in "${address_list[@]}"; do
  endpoint="http://$address:18888/v1/system/upgrade?ota_url=http://$ota_host:$ota_api_port/v1/ota/download"
  echo "Run OTA upgrade for : $address" "$endpoint"
  curl -X POST -m 5 "$endpoint"
done


