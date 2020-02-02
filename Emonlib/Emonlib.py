"""
Author: Alison Almeida
Description:  An asynchronous version of the emonlib library for the micropython firmware on ESP32.

Version: 1.0.0
Date: 02.01.2020
"""

__version__ = "1.0.0"

import machine
import time

from math import pow, sqrt

ADC_BITS = 12
ADC_COUNTS = 1 << ADC_BITS
SUPPLY_VOLTAGE = 3300


class Emonlib(object):
    def __init__(self):
        # public variables
        self.realPower = float()
        self.apparentPower = float()
        self.powerFactor = float()
        self.voltageRms = float()
        self.currentRms = float()

        # privates variables
        self._inPinV = int()
        self._inPinI = int()
        self._phaseCalibration = float()

        self._voltageCalibration = float()
        self._currentCalibration = float()
        self._sampleVoltage = int()
        self._sampleCurrent = int()

        self._lastFilteredVoltage = float()
        self._filteredVoltage = float()
        self._filteredCurrent = float()
        self._offsetVoltage = float()
        self._offsetCurrent = float()

        self._phaseShiftedVoltage = float()
        self._sqrtVoltage = float()
        self._sumVoltage = float()
        self._sqrtCurrent = float()
        self._sumCurrent = float()
        self._instPower = float()
        self._sumPower = float()

        self._startVoltage = int()
        self._lastVoltageCross = bool()
        self._checkVoltageCross = bool()

    async def voltage(self, _in_pin_voltage: int, _voltage_cal: float, _phase_cal: float):
        """
        Sets the pins to be used for voltage sensors
        :param _in_pin_voltage:
        :param _voltage_cal:
        :param _phase_cal:
        :return:
        """

        self._inPinV = _in_pin_voltage
        self._voltageCalibration = _voltage_cal
        self._phaseCalibration = _phase_cal
        self._offsetVoltage = ADC_COUNTS >> 1

    async def current(self, _in_pin_current, _current_cal: float):
        """
        Sets the pins to be used for current sensors
        :param _in_pin_current:
        :param _current_cal:
        :return:
        """
        self._inPinI = _in_pin_current
        self._currentCalibration = _current_cal
        self._offsetCurrent = ADC_COUNTS >> 1

    async def calc_voltage_current(self, crossings: int, timeout: int):
        """
        Calculates realPower,apparentPower,powerFactor, Vrms, Irms, kWh increment
        From a sample window of the mains AC voltage and current.
        The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
        :param crossings:
        :param timeout:
        :return:
        """

        cross_count = 0
        number_samples = 0

        start = time.time()
        adc_voltage = machine.ADC(self._inPinV)
        adc_current = machine.ADC(self._inPinI)

        while True:
            self._startVoltage = adc_voltage.read()
            if ADC_COUNTS * 0.55 > self._startVoltage > ADC_COUNTS * 0.45:
                break
            if time.time() - start > timeout:
                break

        start = time.time()
        while cross_count < crossings and time.time() - start < timeout:
            number_samples += 1
            self._lastFilteredVoltage = self._filteredVoltage

            sample_voltage = adc_voltage.read()
            sample_current = adc_current.read()

            self._offsetVoltage = self._offsetVoltage + ((sample_voltage - self._offsetVoltage) / 1024)
            self._filteredVoltage = sample_voltage - self._offsetVoltage
            self._offsetCurrent = self._offsetCurrent + ((sample_current - self._offsetCurrent) / 1024)
            self._filteredCurrent = sample_current - self._offsetCurrent

            self._sqrtVoltage = pow(self._filteredVoltage, 2)
            self._sumVoltage = self._sqrtVoltage

            self._sqrtCurrent = pow(self._filteredCurrent, 2)
            self._sumCurrent = self._sqrtCurrent

            self._phaseShiftedVoltage = self._lastFilteredVoltage + \
                                        self._phaseCalibration * (self._filteredVoltage - self._lastFilteredVoltage)

            self._instPower = self._phaseShiftedVoltage * self._filteredVoltage
            self._sumPower += self._instPower

            self._lastVoltageCross = self._checkVoltageCross
            if sample_voltage > self._startVoltage:
                self._checkVoltageCross = True
            else:
                self._checkVoltageCross = False

            if number_samples == 1:
                self._lastVoltageCross = self._checkVoltageCross

            if self._lastVoltageCross != self._checkVoltageCross:
                cross_count += 1

        voltage_ratio = self._voltageCalibration * ((SUPPLY_VOLTAGE / 1000) / ADC_COUNTS)
        self.voltageRms = round(voltage_ratio * sqrt(self._sumVoltage / number_samples), 3)

        current_ratio = self._currentCalibration * ((SUPPLY_VOLTAGE / 1000) / ADC_COUNTS)
        self.currentRms = round(current_ratio * sqrt(self._sumCurrent / number_samples), 3)

        self.realPower = round(voltage_ratio * current_ratio * self._sumPower / number_samples, 3)
        self.apparentPower = round(self.voltageRms * self.currentRms, 3)
        self.powerFactor = round(self.realPower / self.apparentPower, 3)

        self._sumVoltage = 0
        self._sumCurrent = 0
        self._sumPower = 0

    async def calc_irms(self, _number_samples: int):
        adc_current = machine.ADC(self._inPinI)

        for sample in range(0, _number_samples):
            pass
