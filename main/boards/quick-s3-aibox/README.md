# QUICK AI模块

# 特性
* 支持双屏
* 基于果云面包板扩展双LCD和马达控制


## 编译配置命令

**配置编译目标为 ESP32S3：**

```bash
idf.py set-target esp32s3
```

**打开 menuconfig：**

```bash
idf.py menuconfig
```

**选择板子：**

```
Xiaozhi Assistant -> Board Type -> QUICK AI智能模块
Xiaozhi Assistant -> Board Type -> LCD Type -> GC9A01
```

**编译：**

```bash
idf.py build
```

**烧录：**
```bash
idf.py flash
```