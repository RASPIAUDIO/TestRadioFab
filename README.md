# Mode d'emploi 

pour gérer l'activation du test de fabrication implanté dans une partition différente (OTA1) de l'application elle même (OTA0)

## Le partitionnement ==> partitions.csv

## code pour l'application :

   if(gpio_get_level(CLICK2) == 0) 
   
   {   
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);        
    if (partition != NULL) {    
      esp_ota_set_boot_partition(partition);      
      esp_restart();      
    }    
   }
   
   /////////////////////////////////////////////////

   ## code pour le test :
   
A PLACER AU TOUT DEBUT DU PROGRAMME
   
 const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);        
    if (partition != NULL) {    
      esp_ota_set_boot_partition(partition);      
    }

  ## Processus de production du binaire complet:

  A- compiler/charger le test
  
  B- preserver le binaire (le contenu de OTA0)
  
  => esptool.py -p /dev/ttyACM0 -b 921600 read_flash 0x10000 0x330000 fab.bin
  
  C- compiler/charger l'application (dans OTA0)
  
  D- charger le test dans OTA1
  
  => esptool.py -p /dev/ttyACM0 -b 921600 write_flash 0x340000 fab.bin
  
  E- sauvegarder l'ensemble
  
  => esptool.py -p /dev/ttyACM0 -b 921600 read_flash 0 0x800000 appF.bin
  
  
   
    
