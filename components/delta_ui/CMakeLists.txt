# ============= components/gui_app/CMakeLists.txt =============

# 设置组件依赖（LVGL 是必须的）
set(REQUIRES lvgl)

# 收集所有源文件
file(GLOB_RECURSE GUI_SOURCES
        # 组件自身源文件
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        # GUI-Guider 生成的核心代码
        ${CMAKE_CURRENT_SOURCE_DIR}/generated/*.c
        # 图片资源
        ${CMAKE_CURRENT_SOURCE_DIR}/images/*.c
        # 自定义字体
        ${CMAKE_CURRENT_SOURCE_DIR}/fonts/*.c
)

# 注册组件
idf_component_register(
        SRCS          ${GUI_SOURCES}
        INCLUDE_DIRS
        "."                     # 组件根目录
        "include"               # 公开头文件
        "generated"             # GUI-Guider 头文件目录
        "images"                # 图片头文件目录
        "fonts"                 # 字体头文件目录
        REQUIRES      ${REQUIRES}
)