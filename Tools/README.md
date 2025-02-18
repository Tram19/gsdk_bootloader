# Gremsy MKFW

`gremsy_mkfw` là công cụ dùng để tạo ra file image chứa thông tin của firmware (FW) cần nạp. File image này bao gồm các thông tin sau:

- `gimbal_id`    = ID của gimbal  
- `gimbal_name`  = Tên của gimbal  
- `version`      = Phiên bản firmware  
- `description`  = Mô tả của firmware  
- `build_time`   = Thời gian build firmware  
- `image`        = Nội dung file firmware  
- `image_size`   = Kích thước file firmware  
- `crc32`        = CRC32 checksum của file firmware  

## Cách Sử Dụng

Để tạo file image chứa thông tin firmware, hãy chạy lệnh sau:

```
python gremsy_mkfw.py --gimbal_id <gimbal_id> --gimbal_name <gimbal_name> --version <version> --image <firmware_path>
```
## Cách Sử Dụng

Giả sử có firmware tại đường dẫn /FW/gremsyZio_v785_Beta_boot_compatible.bin và muốn tạo file image cho gimbal có:

gimbal_id: 51
gimbal_name: Vio
version: 785
Thì lệnh thực hiện sẽ như sau:
```
python gremsy_mkfw.py --gimbal_id 51 --gimbal_name Vio --version 785 --image /FW/gremsyZio_v785_Beta_boot_compatible.bin
```