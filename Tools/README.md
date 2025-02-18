# Gremsy MKFW

`gremsy_mkfw` là công cụ dùng để tạo ra file image chứa thông tin của firmware (FW) cần nạp. File image này bao gồm các thông tin sau:

- `proto['gimbal_id']`    = 0  
- `proto['gimbal_name']`  = ""  
- `proto['version']`      = ""  
- `proto['description']`  = ""  
- `proto['build_time']`   = 0  
- `proto['image']`        = ""  
- `proto['image_size']`   = 0  
- `proto['crc32']`        = 0  

## Cách Sử Dụng

Để tạo file image chứa thông tin firmware, hãy chạy lệnh sau:

```bash
python gremsy_mkfw.py --gimbal_id <gimbal_id> --gimbal_name <gimbal_name> --version <version> --image <firmware_path>

## Cách Sử Dụng

Giả sử có firmware tại đường dẫn /FW/gremsyZio_v785_Beta_boot_compatible.bin và muốn tạo file image cho gimbal có:

gimbal_id: 51
gimbal_name: Vio
version: 785
Thì lệnh thực hiện sẽ như sau:
```bash
python gremsy_mkfw.py --gimbal_id 51 --gimbal_name Vio --version 785 --image /FW/gremsyZio_v785_Beta_boot_compatible.bin
