from setuptools import setup, find_packages

setup(
    name='BlockFlow',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        # List your package's dependencies here
    ],
    author='Quincy Jones',
    author_email='quincy@implementedrobotics.com',
    description='A short description of your package',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/implementedrobotics/block_flow',
    license='MIT',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
    ],
)
