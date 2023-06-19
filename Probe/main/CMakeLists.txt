idf_component_register(
    SRCS "app.cpp" "main.cpp" 
         "../components/LoRa/lora.c"
          "libs/SHT2x/SHT2x.cpp"
          "libs/BMP180/src/BMP180.cpp"
          "libs/TinyGPS++/src/TinyGPS++.cpp"
    INCLUDE_DIRS "" 
                 "../components/LoRa/" "libs"
    )
