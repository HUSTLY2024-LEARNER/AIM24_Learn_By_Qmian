#include <LangYa/CodeHelper.hpp>

using namespace LangYa;

namespace {
	struct XYZ {
		float X{0};
		float Y{0};
		float Z{0};

		std::ostream& Format(std::ostream& stream) const noexcept {
			return stream << '[' << X << ", " << Y << ", " << Z << ']';
		}

		bool Load(const boost::property_tree::ptree& config) noexcept {
			X = config.get<float>("X", 0);
			Y = config.get<float>("Y", 0);
			Z = config.get<float>("Z", 0);
			return true;
		}

		void Save(boost::property_tree::ptree& config) const noexcept {
			config.put("X", X);
			config.put("Y", Y);
			config.put("Z", Z);
		}
	};

	struct Data {
		std::string Text{};
		XYZ Vec{};

		std::ostream& Format(std::ostream& stream) const noexcept {
			stream << "Text: " << Text << ", Vec: ";
			return Vec.Format(stream);
		}

		bool Load(const boost::property_tree::ptree& config) noexcept {
			Text = config.get<std::string>("Text", "");
			return Vec.Load(config.get_child("Vec"));
		}

		void Save(boost::property_tree::ptree& config) const noexcept {
			config.put("Text", Text);

			boost::property_tree::ptree vec{};
			Vec.Save(vec);
			config.put_child("Vec", vec);
		}
	};
}


int main() {
	boost::property_tree::ptree config{};
	if (!DefaultLoad(config, "config.json")) return 1;
	Data data{};
	if (!data.Load(config)) return 1;
	std::cout << data << '\n';
	data.Text += data.Text;
	config << data;
	return DefaultSave(config, "config2.json") ? 0 : 1;
}