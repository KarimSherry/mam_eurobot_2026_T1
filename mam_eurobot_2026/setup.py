from setuptools import setup, find_packages

# package_name = 'mam_eurobot_2026*'
package_name = 'mam_eurobot_2026'

setup(
    name=package_name,
    version='0.0.1',
    # packages=find_packages(where='.', include=[package_name]),
    packages=find_packages(where='.', include=[f'{package_name}', f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Freddy Liendo',
    maintainer_email='liendomf@univ-smb.fr',
    description='This is a template package for the Master Advanced Mechatronics teams preparing for the Eurobot 2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'py_test = mam_eurobot_2026.py_test:main',
            'aruco_detector = mam_eurobot_2026.vision.aruco_detector:main',
            'color_detector = mam_eurobot_2026.vision.color_detector:main'
        ],
    },
)
