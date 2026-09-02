from setuptools import setup, find_packages

setup(
    name="targetless-calibration",
    version="1.0.0",
    author="Gemini Notebook Collaborative Developer",
    description="Targetless single-shot 3D camera-to-robot calibration tool based on 3-point registration.",
    long_description_content_type="text/markdown",
    url="https://github.com/iammohith/Targetless-Hand-Eye-Calibration",
    packages=find_packages(),
    py_modules=["targetless_calibration"],
    install_requires=[
        "numpy>=1.20.0",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering",
    ],
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "targetless-calibrate=targetless_calibration:main",
        ],
    },
)
