#include <iostream>
#include <fstream>
#include <iomanip>
#include <LangYa/AutoAim/Gimbals.hpp>

using namespace LangYa;

int main() {
	Cango::TypedMessage<sizeof(GameData)> message{};

	std::ifstream stream{"data.txt"};
	auto& data = message.GetDataAs<GameData>();

	bool my_team_red;
	stream >> my_team_red;
	data.GameCode.IsMyTeamRed = my_team_red; // 1

	std::uint16_t soh;
	stream >> soh;
	data.GameCode.SelfOutpostHealth = soh; // 2

	std::uint16_t eoh;
	stream >> eoh;
	data.GameCode.EnemyOutpostHealth = eoh; // 3

	bool hp;
	stream >> hp;
	data.GameCode.HeroPrecaution = hp; // 4

	bool igb;
	stream >> igb;
	data.GameCode.IsGameBegin = igb; // 5

	bool irh;
	stream >> irh;
	data.GameCode.IsReturnedHome = irh; // 6

	stream >> data.SelfHealth; // 7
	stream >> data.AmmoLeft; // 8
	stream >> data.TimeLeft; // 9
	stream >> data.UWBPosition.X; // 10
	stream >> data.UWBPosition.Y; // 11

	message.Type = GameData::TypeID;

	for (const auto byte: message.ToSpan()) {
		std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << ' ';
	}
	std::cout << std::endl;

	return 0;
}
