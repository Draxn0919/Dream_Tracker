from setuptools import setup

package_name = 'Dream_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',       
        'mediapipe',           
        'rclpy',               
        'numpy',              
    ],
    zip_safe=True,
    maintainer='Daniel Rivera',
    maintainer_email='danrivera505@gmail.com',
    description='blink, yawn and microsleep detector, using OpenCV, FaceMesh and ROS2.',
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publi = proyecto_final.img_publisher:main',
            'img_microdreams = proyecto_final.img_microdreams:main',
            'img_emo = proyecto_final.img_emotion:main',
            'img_tracking = proyecto_final.img_eyes_tracking:main',
            'img_displa = proyecto_final.img_display:main',
        ],
    },
)
