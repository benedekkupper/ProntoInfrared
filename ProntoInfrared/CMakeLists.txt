target_sources(${PROJECT_NAME} PRIVATE
    infrared_receiver.cpp
    infrared_transmitter.cpp
    main.cpp
    pronto_hex.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
