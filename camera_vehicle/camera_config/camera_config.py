#camera_config.py

#

#
#


#

#

import cv2
from abc import ABC, abstractmethod
#from color_detection import color_detection_mask
from color_detection import color_detection_mask
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np

class video_capture_abs(ABC):
    @abstractmethod
    def start_camera(self):
        pass

    @abstractmethod
    def stop_camera(self):
        pass

    @abstractmethod
    def camera_visualization(self):
        pass

class video_capture(video_capture_abs):
    def __init__(self, camera):
        self.camera = camera
        self.enabled = False

    def start_camera(self):
        self.enabled = True
        self.camera_visualization()

    def stop_camera(self):
        self.enabled = False

class video_capture_with_filters(video_capture):
    def __init__(self, camera, filter):
        self.desired_shape = 0
        super().__init__(camera)
        self.filter = filter
        self.contrast = 1.0  # Valor inicial de contraste

        self.filters = {
            "Gray": cv2.COLOR_BGR2GRAY,
            "HSV": cv2.COLOR_BGR2HSV,
        }
        self.filter_keys = {
            "w": "HSV",
            "h": "HSV",
            "n": "None"
        }
        self.color_masks = {
            "r": "red",
            "b": "blue",
            "g": "green",
            "y": "yellow",
            "p": "pink"
        }

        # === GUI Tkinter ===
        self.root = tk.Tk()
        self.root.title("Camera Feed")

        self.label = tk.Label(self.root)
        self.label.pack()
        
        
        # slider y boton de filtro mediano
        self.median_enabled = False
        self.kernel_size = 3
        
        self.saturation = 1.0        
        # Slider de saturación
        self.slider_s = tk.Scale(self.root, from_=0.0, to=3.0, resolution=0.1,
                                 orient=tk.HORIZONTAL, label="Saturación",
                                 command=self.update_saturation)
        self.slider_s.set(self.saturation)
        self.slider_s.pack(pady=5)

        # Slider de contraste
        self.slider = tk.Scale(self.root, from_=0.1, to=3.0, resolution=0.1,
                               orient=tk.HORIZONTAL, label="Contraste",
                               command=self.update_contrast)
        self.slider.set(self.contrast)
        self.slider.pack(pady=5)
        
        #SLIDER Y BOTON DE MEDIANA
        self.median_button = tk.Button(self.root, text="Activar filtro mediana",
                                       command=self.toggle_median)
        self.median_button.pack(pady=5)

        # Slider de tamaño de kernel
        self.kernel_slider = tk.Scale(self.root, from_=1, to=17, resolution=2,
                                      orient=tk.HORIZONTAL, label="Tamaño de kernel",
                                      command=self.update_kernel)
        self.kernel_slider.set(self.kernel_size)
        self.kernel_slider.pack(pady=5)
        

        self.root.bind("<Key>", self.key_handler)
    def update_saturation(self, val):
        self.saturation = float(val)
    def update_contrast(self, val):
        self.contrast = float(val)
        
    def toggle_median(self):
        self.median_enabled = not self.median_enabled
        if self.median_enabled:
            self.median_button.config(text="Desactivar filtro mediana")
        else:
            self.median_button.config(text="Activar filtro mediana")

    def update_kernel(self, val):
        k = int(val)
        if k % 2 == 0:
            k += 1  # forzar impar
        self.kernel_size = k
        
        
    def key_handler(self, event):
        key = event.char.lower()
        if key == 'q':
            self.stop_camera()
            self.root.quit()
        elif key in self.filter_keys:
            self.filter = self.filter_keys[key]
        elif key in self.color_masks:
            self.filter = self.color_masks[key]
        elif key in ['3','4', '7']:
            self.desired_shape = int(key)

    def camera_visualization(self):
        def update_frame():
            if not self.enabled:
                return

            ret, frame = self.camera.read()
            if not ret or frame is None:
                print("[CAMERA] Error al leer el frame")
                self.root.after(10, update_frame)
                return
            frame = cv2.resize(frame, (500, 300))
            

            # === Aplicar contraste ANTES de cualquier filtro ===
            frame = cv2.convertScaleAbs(frame, alpha=self.contrast, beta=0)

            if self.filter != "None":
                if self.filter in self.filters:
                    frame = self.change_filter(frame)
                elif self.filter in self.color_masks.values():
                    self.detector = color_detection_mask(frame, self.filter, self.desired_shape)
                    frame = self.detector.color_mask()
            

                
            # SATURATION
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[..., 1] *= self.saturation
            hsv[..., 1] = np.clip(hsv[..., 1], 0, 255)
            frame = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
            
            # filtro mediano
            
            if self.median_enabled:
                frame = cv2.medianBlur(frame, self.kernel_size)
                
            # === cualquier filtro ===
            frame = cv2.resize(frame, (500, 300))
            # Asegurar formato RGB para Tkinter
            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            else:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            self.label.imgtk = imgtk
            self.label.config(image=imgtk)

            self.root.after(10, update_frame)

        update_frame()
        self.root.mainloop()

    def change_filter(self, frame):
        return cv2.cvtColor(frame, self.filters[self.filter])