if(NOT DEFINED SOURCE_DIR)
  message(FATAL_ERROR "SOURCE_DIR is required")
endif()
if(NOT DEFINED BINARY_DIR)
  message(FATAL_ERROR "BINARY_DIR is required")
endif()
if(NOT DEFINED PYTHON_EXECUTABLE)
  set(PYTHON_EXECUTABLE python)
endif()
set(smoke_output "${BINARY_DIR}/render-smoke-output")
file(MAKE_DIRECTORY "${smoke_output}")
file(COPY "${SOURCE_DIR}/cases/IFAmpliflier_min3/output/placement.txt" DESTINATION "${smoke_output}")
file(COPY "${SOURCE_DIR}/cases/IFAmpliflier_min3/output/routing.txt" DESTINATION "${smoke_output}")
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" "${SOURCE_DIR}/tools/render_layout.py"
          --input "${SOURCE_DIR}/cases/IFAmpliflier_min3/input"
          --output "${smoke_output}"
          --name render_smoke
          --dpi 80
  RESULT_VARIABLE render_result
)
if(NOT render_result EQUAL 0)
  message(FATAL_ERROR "render smoke test failed with exit code ${render_result}")
endif()
set(png "${smoke_output}/render_smoke_layout.png")
if(NOT EXISTS "${png}")
  message(FATAL_ERROR "missing render smoke PNG: ${png}")
endif()
file(SIZE "${png}" size)
if(size EQUAL 0)
  message(FATAL_ERROR "empty render smoke PNG: ${png}")
endif()
