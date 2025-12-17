#pragma once

// --- Standard Library Includes ---
#include <cstdlib>		// For std::abs
#include <vector>		// For std::vector
#include <memory>		// For std::shared_ptr, std::make_shared
#include <algorithm>	// For std::any_of, std::all_of

// --- PROS/VEX Library Includes ---
#include "pros/misc.h"	// For pros::v5::MotorBrake, etc.
#include "main.h"		// General project header (assuming it defines Aliases)

// --- Local Library Includes ---
#include "libs/include/T_Lib/Motor.hpp" // T_Motor class definition

using namespace T_Lib;

/**
 * @brief Manages a collection of T_Motor objects, providing a unified interface
 * for setting control parameters (RPM, Brake Mode, PID constants) across all motors simultaneously.
 * * Uses std::shared_ptr for robust memory management of the T_Motor objects.
 */
class T_MotorGroup {
private:
	/// @brief Vector of shared pointers to T_Motor objects in this group.
	std::vector<std::shared_ptr<T_Motor>> motors;

	// --- Private Conversion Helpers for Constructors ---

	/**
	 * @brief Converts a vector of raw T_Motor pointers into shared pointers.
	 * The shared pointers use a custom deleter to avoid managing the lifetime
	 * of the raw pointers (assuming they are owned elsewhere).
	 */
	static std::vector<std::shared_ptr<T_Motor>> convertRawPtrs(const std::vector<T_Motor*>& rawPtrs) {
		std::vector<std::shared_ptr<T_Motor>> result;
		for (auto* ptr : rawPtrs)
			// Use a custom deleter: [](T_Motor*) {} which does nothing.
			result.emplace_back(std::shared_ptr<T_Motor>(ptr, [](T_Motor*) {}));
		return result;
	}

	/// @brief Overload for initializer list of raw pointers.
	static std::vector<std::shared_ptr<T_Motor>> convertRawPtrs(std::initializer_list<T_Motor*> rawPtrs) {
		return convertRawPtrs(std::vector<T_Motor*>(rawPtrs));
	}

	/**
	 * @brief Converts a vector of T_Motor objects into shared pointers.
	 * This copies the original T_Motor objects into new shared pointers.
	 */
	static std::vector<std::shared_ptr<T_Motor>> convertObjs(const std::vector<T_Motor>& objs) {
		std::vector<std::shared_ptr<T_Motor>> result;
		for (const auto& obj : objs)
			// Deep copy the T_Motor object
			result.emplace_back(std::make_shared<T_Motor>(obj));
		return result;
	}

	/// @brief Overload for initializer list of T_Motor objects.
	static std::vector<std::shared_ptr<T_Motor>> convertObjs(std::initializer_list<T_Motor> objs) {
		return convertObjs(std::vector<T_Motor>(objs));
	}

public:
	// --- Constructors: Initialization from Existing Motor Objects/Pointers ---

	/// @brief Constructs from an initializer list of shared pointers.
	explicit T_MotorGroup(std::initializer_list<std::shared_ptr<T_Motor>> list) : motors(list) {}
	
	/// @brief Constructs from a constant vector of shared pointers.
	explicit T_MotorGroup(const std::vector<std::shared_ptr<T_Motor>>& list) : motors(list) {}
	
	/// @brief Constructs by moving a vector of shared pointers.
	explicit T_MotorGroup(std::vector<std::shared_ptr<T_Motor>>&& list) : motors(std::move(list)) {}
	
	/// @brief Constructs from an initializer list of raw T_Motor pointers (non-owning).
	explicit T_MotorGroup(std::initializer_list<T_Motor*> list) : motors(convertRawPtrs(list)) {}
	
	/// @brief Constructs from a constant vector of raw T_Motor pointers (non-owning).
	explicit T_MotorGroup(const std::vector<T_Motor*>& list) : motors(convertRawPtrs(list)) {}
	
	/// @brief Constructs from an initializer list of T_Motor objects (deep copies).
	explicit T_MotorGroup(std::initializer_list<T_Motor> list) : motors(convertObjs(list)) {}
	
	/// @brief Constructs from a constant vector of T_Motor objects (deep copies).
	explicit T_MotorGroup(const std::vector<T_Motor>& list) : motors(convertObjs(list)) {}

	// --- Constructors: Initialization from Port Numbers ---

	/**
	 * @brief Constructs the group from an initializer list of integer port numbers.
	 * @param ports List of port numbers. Negative ports indicate reversed direction.
	 * @param gearset The motor gear cartridge type (default: Aliases::Blue).
	 */
	explicit T_MotorGroup(std::initializer_list<int> ports, pros::v5::MotorGears gearset = Aliases::Blue) {
		for (int port : ports) {
			bool reversed = (port < 0);
			int absPort = std::abs(port);
			motors.emplace_back(std::make_shared<T_Motor>(absPort, gearset, reversed));
		}
	}

	/**
	 * @brief Constructs the group from a vector of integer port numbers.
	 * @param ports Vector of port numbers. Negative ports indicate reversed direction.
	 * @param gearset The motor gear cartridge type (default: Aliases::Blue).
	 */
	explicit T_MotorGroup(const std::vector<int>& ports, pros::v5::MotorGears gearset = Aliases::Blue) {
		for (int port : ports) {
			bool reversed = (port < 0);
			int absPort = std::abs(port);
			motors.emplace_back(std::make_shared<T_Motor>(absPort, gearset, reversed));
		}
	}

	// =========================================================================
	// === Core Control Functions ===
	// =========================================================================

	/// @brief Sets the target RPM for all motors in the group.
	void setTargetRPM(double rpm) {
		for (auto& m : motors) m->setTargetRPM(rpm);
	}

	/// @brief Sets the target speed as a percentage of max velocity for all motors (e.g., -100 to 100).
	void setTargetPercent(double percent) {
		for (auto& m : motors) m->setTargetPercent(percent);
	}

	/// @brief Commands all motors to stop. The final state depends on the brake mode.
	void stop() {
		for (auto& m : motors) m->stop();
	}

	/// @brief Resets the encoder position of all motors to 0.
	void resetPositions() {
		for (auto& m : motors) m->resetPosition();
	}

	/// @brief Sets the brake mode for all motors in the group.
	void setBrakeMode(pros::v5::MotorBrake mode) {
		for (auto& m : motors) m->setBrakeMode(mode);
	}

	// =========================================================================
	// === Torque and Compensation Functions ===
	// =========================================================================

	/// @brief Enables/disables the minimum torque threshold for all motors.
	void setMinTorque(bool enabled) {
		for (auto& m : motors) m->setMinTorque(enabled);
	}

	/// @brief Enables/disables minimum torque and sets a primary minimum voltage (mV).
	void setMinTorque(bool enabled, int minMv) {
		for (auto& m : motors) m->setMinTorque(enabled, minMv);
	}

	/// @brief Enables/disables minimum torque and sets both primary and low-speed minimum voltages (mV).
	void setMinTorque(bool enabled, int minMv, int minLowMv) {
		for (auto& m : motors) m->setMinTorque(enabled, minMv, minLowMv);
	}

	/// @brief Enables/disables load compensation (velocity feedforward adjustment).
	void setLoadCompensation(bool enabled) {
		for (auto& m : motors) m->setLoadCompensation(enabled);
	}

	/// @brief Enables/disables load compensation and sets a boost factor (kBoost).
	void setLoadCompensation(bool enabled, double kBoost) {
		for (auto& m : motors) m->setLoadCompensation(enabled, kBoost);
	}

	/// @brief Enables/disables load compensation, sets kBoost, and sets the low-current threshold (mA).
	void setLoadCompensation(bool enabled, double kBoost, int threshold) {
		for (auto& m : motors) m->setLoadCompensation(enabled, kBoost, threshold);
	}

	// =========================================================================
	// === PID Constant Functions ===
	// =========================================================================

	/// @brief Sets the velocity (kv) and proportional (kp) constants for both low and high speed PID.
	void setDualConstants(double kvLow, double kpLow, double kvHigh, double kpHigh) {
		for (auto& m : motors) m->setDualConstants(kvLow, kpLow, kvHigh, kpHigh);
	}

	/// @brief Sets the velocity (kv) and proportional (kp) constants for low speed PID only.
	void setLowConstants(double kvLow, double kpLow) {
		for (auto& m : motors) m->setLowConstants(kvLow, kpLow);
	}

	/// @brief Sets the velocity (kv) and proportional (kp) constants for high speed PID only.
	void setHighConstants(double kvHigh, double kpHigh) {
		for (auto& m : motors) m->setHighConstants(kvHigh, kpHigh);
	}

	// =========================================================================
	// === Slew Control Functions ===
	// =========================================================================

	/// @brief Enables or disables the slew rate limit mechanism.
	void setSlewLimitEnabled(bool SlewOn){
		for (auto& m : motors) m->setSlewLimitEnabled(SlewOn);
	}

	/// @brief Sets the maximum RPM change allowed per update cycle for slew limiting.
	void setSlewRate(double Slew){
		for (auto& m : motors) m->setSlewRate(Slew);
	}

	// =========================================================================
	// === PID Enable Control Functions ===
	// =========================================================================

	/// @brief Enables or disables the internal PID control loop for all motors.
	void setPIDEnabled(bool enabled) {
		for (auto& m : motors) m->setPIDEnabled(enabled);
	}

	/// @brief Checks if the PID control loop is enabled for AT LEAST one motor in the group.
	bool isAnyPIDEnabled() const {
		return std::any_of(motors.begin(), motors.end(), [](const std::shared_ptr<T_Motor>& m) {
			return m->isPIDEnabled();
		});
	}

	/// @brief Checks if the PID control loop is enabled for ALL motors in the group.
	bool areAllPIDEnabled() const {
		return std::all_of(motors.begin(), motors.end(), [](const std::shared_ptr<T_Motor>& m) {
			return m->isPIDEnabled();
		});
	}

	// =========================================================================
	// === Telemetry & Status Functions ===
	// =========================================================================

	/**
	 * @brief Calculates the average RPM across all motors in the group.
	 * @return The average RPM, or 0 if the group is empty.
	 */
	double getAverageRPM() const {
		double total = 0;
		for (const auto& m : motors) total += m->getRPM(); // Assumes T_Motor::getRPM() exists
		return motors.empty() ? 0 : total / motors.size();
	}

	/// @brief Checks if AT LEAST one motor in the group is currently spinning (non-zero RPM).
	bool isAnySpinning() const {
		return std::any_of(motors.begin(), motors.end(), [](const std::shared_ptr<T_Motor>& m) {
			return m->isSpinning(); // Assumes T_Motor::isSpinning() exists
		});
	}

	/**
	 * @brief Gets a constant reference to the underlying vector of shared motors.
	 * @return A const reference to the vector<shared_ptr<T_Motor>>.
	 */
	const std::vector<std::shared_ptr<T_Motor>>& getMotors() const { return motors; }
	
	/**
	 * @brief Gets the number of motors in the group. (New Getter)
	 * @return The size of the motor group.
	 */
	size_t size() const { return motors.size(); }
};