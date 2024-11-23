from setuptools import find_packages, setup

package_name = 'primeiro_pacote'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/meu_primeiro_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Ribeiro',
    maintainer_email='alexfilho2004@icloud.com',
    description='Exemplo de pacote',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teste = primeiro_pacote.meu_primeiro_no:main',
            'talker = primeiro_pacote.talker:main',
            'listener = primeiro_pacote.listener:main',
            'r2d2 = primeiro_pacote.r2d2:main',
            'pid = primeiro_pacote.r2d2_controle:main',
            'vfh = primeiro_pacote.VFH:main',
            'estrela = primeiro_pacote.Astar:main',
            'classe = primeiro_pacote.no_com_classe:main'
        ],
    },
)
