import serial
import time

# Ganti 'COMX' dengan port serial Arduino Anda (e.g. COM3, /dev/ttyACM0)
ARDUINO_PORT = 'COM3'
BAUD_RATE = 9600

try:
    # Membuka koneksi serial
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE)
    time.sleep(2)  # Memberi waktu Arduino untuk melakukan reset

    while True:
        # Data yang akan dikirim
        data_to_send = "Hello, Arduino!\n"  # \n sebagai penanda akhir data
        
        # Mengirim data
        ser.write(data_to_send.encode())
        print(f"Data terkirim: {data_to_send.strip()}")
        
        # Membaca balasan dari Arduino (jika ada)
        if ser.in_waiting > 0:
            received_data = ser.readline().decode().strip()
            print(f"Balasan Arduino: {received_data}")
        
        time.sleep(1)  # Delay antara pengiriman data

except serial.SerialException as e:
    print(f"Error koneksi serial: {e}")
except KeyboardInterrupt:
    print("Program dihentikan oleh user")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Koneksi serial ditutup")