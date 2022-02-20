#ifndef SC_SHUFFLEBOARD_H
#define SC_SHUFFLEBOARD_H

#include "frc/shuffleboard/Shuffleboard.h"

#include "networktables/NetworkTableEntry.h"

#include "wpi/StringMap.h"

#include <cstring>

namespace SC
{
/* ============================================================================== */

	/**
	 * @brief Shuffleboard input (Dashboard -> RoboRIO) object
	 */
	template<class T>
	class SC_SBItem
	{
	public:
		SC_SBItem(std::string Tab, std::string Name, double DefaultVal = 0.0)
		{
			this->_tabName = Tab;
			this->_name = Name;
			this->_tab = &(frc::Shuffleboard::GetTab(Tab));
			// this->_props = nullptr;
			this->_widget = &(this->_tab->Add(Name, DefaultVal));
			this->_entry = this->_widget->GetEntry();
		}

		SC_SBItem(std::string Tab, std::string Name, frc::BuiltInWidgets Widget, double DefaultVal = 0.0)
		{
			this->_tabName = Tab;
			this->_name = Name;
			this->_type = Widget;
			this->_tab = &(frc::Shuffleboard::GetTab(Tab));
			this->_widget = &(this->_tab->Add(Name, DefaultVal).WithWidget(Widget));
			this->_entry = this->_widget->GetEntry();
		}

		SC_SBItem(std::string Tab, std::string Name, frc::BuiltInWidgets Widget, wpi::StringMap<std::shared_ptr<nt::Value>> Props, double DefaultVal = 0.0)
		{
			this->_tabName = Tab;
			this->_name = Name;
			this->_type = Widget;
			this->_props = Props;
			this->_tab = &(frc::Shuffleboard::GetTab(Tab));
			this->_widget = &(this->_tab->Add(Name, DefaultVal).WithWidget(Widget).WithProperties(Props));
			this->_entry = this->_widget->GetEntry();
		}

		~SC_SBItem() { this->_entry.Delete(); }

		void SetProperties(wpi::StringMap<std::shared_ptr<nt::Value>> Props)
		{
			this->_props = Props;
			this->_widget = this->_widget->WithProperties(Props);
		}

		bool GetBool(bool DefaultValue = false) { return this->_entry.GetBoolean(DefaultValue); }

		double GetNumber(double DefaultValue = 0.0) { return this->_entry.GetDouble(DefaultValue); }

		void SetBool(bool value) { this->_entry.SetBoolean(value); }

		void SetNumber(double value) { this->_entry.SetDouble(value); }
				
		void SetText(std::string value) { this->_entry.SetString(value); }

	private:
		wpi::StringMap<std::shared_ptr<nt::Value>> _props = { };
		std::string _tabName = "SmartDashboard";
		std::string _name = "";
		frc::BuiltInWidgets _type;
		frc::ShuffleboardTab *_tab;
		frc::SimpleWidget *_widget;
		nt::NetworkTableEntry _entry;
	};

/* ============================================================================== */

	/**
	 * @brief Shuffleboard layout helper class.
	 */
	class SC_SBLayout
	{
	public:
		SC_SBLayout() { }
		~SC_SBLayout() { }

	private:

	};

/* ============================================================================== */

	/**
	 * @brief Shuffleboard builder wrapper class
	 */
	class SC_SBBuilder
	{
	public:
		SC_SBBuilder(std::string TabName = "SmartDashboard") { this->_tabName = TabName; }
		~SC_SBBuilder() { }

		void SetTab(std::string TabName) { this->_tabName = TabName; }

		SC::SC_SBItem<bool>* AddBoolean(std::string Name) { return new SC::SC_SBItem<bool>(this->_tabName, Name, frc::BuiltInWidgets::kBooleanBox); }
		SC::SC_SBItem<bool>* AddBoolean(std::string Name, wpi::StringMap<std::shared_ptr<nt::Value>> Props) { return new SC::SC_SBItem<bool>(this->_tabName, Name, frc::BuiltInWidgets::kBooleanBox, Props); }

		SC::SC_SBItem<double>* AddNumBar(std::string Name, double DefaultVal = 0.0) { return new SC::SC_SBItem<double>(this->_tabName, Name, frc::BuiltInWidgets::kNumberBar, DefaultVal); }
		SC::SC_SBItem<double>* AddNumBar(std::string Name, wpi::StringMap<std::shared_ptr<nt::Value>> Props, double DefaultVal = 0.0) { return new SC::SC_SBItem<double>(this->_tabName, Name, frc::BuiltInWidgets::kNumberBar, Props, DefaultVal); }

		SC::SC_SBItem<std::string>* AddText(std::string Name) { return new SC::SC_SBItem<std::string>(this->_tabName, Name, frc::BuiltInWidgets::kTextView); }

		template<class T>
		SC::SC_SBItem<T>* AddEntry(std::string Name, double DefaultVal = 0.0) { return new SC::SC_SBItem<T>(this->_tabName, Name, DefaultVal); }
		// SC::SC_SBInput<int>* AddComboBoxInput();
		
		// SC::SC_SBOutput<bool>* AddBooleanOutput(std::string Name);
		// SC::SC_SBOutput<double>* AddNumericOutput();

		SC::SC_SBLayout* CreateLayout() { return new SC::SC_SBLayout(); }
		
	private:
		std::string _tabName;
	};
}

#endif // SC_SHUFFLEBOARD_H