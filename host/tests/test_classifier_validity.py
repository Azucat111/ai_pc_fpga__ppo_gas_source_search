import sys
import unittest
from pathlib import Path


HOST = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST / 'app' / 'software'))

from car.workflow import classify_gas10_scan


class Predictor:
    def predict(self, _values):
        return {
            'class_key': 'alcohol',
            'probabilities': {'air': 0.01, 'alcohol': 0.99},
            'model_profile_equivalence_verified': False,
            'classification_valid': False,
        }


class ClassificationValidityTests(unittest.TestCase):
    def test_unverified_profile_keeps_inference_invalid(self):
        scan = {
            'boot_id': 7,
            'seq': 3,
            'scan_id': 2,
            'scan_end_ms': 1000,
            'acquisition_valid': True,
            'gas_valid_mask': 1023,
            'gas10_ohm': [1000] * 10,
        }
        frame = {'boot_id': 7, 'timestamp_ms': 1100}

        result = classify_gas10_scan(scan, frame, Predictor())

        self.assertEqual(result['status'], 'inference_unverified_profile')
        self.assertEqual(result['class_key'], 'alcohol')
        self.assertEqual(result['probabilities']['alcohol'], 0.99)
        self.assertFalse(result['classification_valid'])
        self.assertFalse(result['model_profile_equivalence_verified'])


if __name__ == '__main__':
    unittest.main()
