from __future__ import annotations

import os

import onnxruntime as ort


def get_policy_ort_providers() -> list[str]:
    """Return stable ONNXRuntime providers for onboard policy inference."""

    disabled = {
        provider.strip()
        for provider in os.environ.get("INSTINCT_ONNX_DISABLE_PROVIDERS", "TensorrtExecutionProvider").split(",")
        if provider.strip()
    }
    preferred = [
        "CUDAExecutionProvider",
        "CPUExecutionProvider",
    ]
    available = set(ort.get_available_providers())
    providers = [provider for provider in preferred if provider in available and provider not in disabled]
    if providers:
        return providers
    return [provider for provider in ort.get_available_providers() if provider not in disabled]
