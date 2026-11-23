"""Garante que os programas do rover compilam (sem precisar de câmera, YOLO ou torch)."""
import pathlib
import py_compile

import pytest

SRC = pathlib.Path(__file__).resolve().parents[1] / "src"


@pytest.mark.parametrize("arquivo", sorted(SRC.glob("*.py")), ids=lambda p: p.name)
def test_compila(arquivo):
    py_compile.compile(str(arquivo), doraise=True)
