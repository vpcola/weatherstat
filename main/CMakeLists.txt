idf_component_register(
	SRCS "main.cpp" "CayenneLPP.cpp"
    INCLUDE_DIRS "."
    REQUIRES ttn-esp32 sh2x plantower5003 stc3100)
