#include <atomic>
#include <chrono>
#include <concepts>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

// 这个文件确定了一些基本语法

namespace {
	template <typename TCount>
	class Counter {
		TCount current{};
		TCount maxCount{};

	public:
		explicit Counter(const TCount maxCount = 5) : maxCount(maxCount) {}

		[[nodiscard]] const TCount& Current() const noexcept { return current; }

		[[nodiscard]] TCount& Current() noexcept { return current; }

		[[nodiscard]] const TCount& MaxCount() const noexcept { return maxCount; }

		[[nodiscard]] TCount& MaxCount() noexcept { return maxCount; }

		[[nodiscard]] bool IsReached() const noexcept { return current >= maxCount; }

		[[nodiscard]] bool Count() noexcept {
			if (current < maxCount - 1) {
				++current;
				return false;
			}
			current = maxCount;
			return true;
		}

		void Reset() noexcept { current = 0; }
	};

	struct TaskArchitect {
		virtual ~TaskArchitect() noexcept = default;
		[[nodiscard]] virtual bool Inspect() const noexcept = 0;
		virtual void Execute() noexcept = 0;
		[[nodiscard]] virtual bool IsDone() const noexcept = 0;
		virtual void Interrupt() noexcept = 0;
		virtual void Reset() noexcept = 0;
	};

	class DefaultTaskArchitect : TaskArchitect {
		std::atomic_bool done{};

	public:
		[[nodiscard]] bool IsDone() const noexcept final { return done.load(std::memory_order::relaxed); }

		void Interrupt() noexcept final { done.store(true, std::memory_order::relaxed); }

		void Reset() noexcept final { done.store(false, std::memory_order::relaxed); }
	};

	template <typename TArchitect>
	concept IsTaskArchitect = requires(TArchitect& architect) {
		{ architect.Inspect() } -> std::convertible_to<bool>;
		{ architect.Execute() };
		{ architect.IsDone() } -> std::convertible_to<bool>;
		{ architect.Interrupt() };
		{ architect.Reset() };
	};

	template <typename TItem>
	struct SourceNode {
		using ItemType = TItem;
		virtual ~SourceNode() noexcept = default;
		[[nodiscard]] virtual bool GetItem(TItem& item) noexcept = 0;
	};

	template <typename TNode, typename TItem = typename TNode::ItemType>
	concept IsSourceNode = requires(TNode& node, TItem& item) {
		{ node.GetItem(item) } -> std::convertible_to<bool>;
	};

	template <typename TItem>
	struct DestinationNode {
		using ItemType = TItem;
		virtual ~DestinationNode() noexcept = default;
		virtual void SetItem(const TItem& item) noexcept = 0;
	};

	template <typename TNode, typename TItem = typename TNode::ItemType>
	concept IsDestinationNode = requires(TNode& node, TItem& item) {
		{ node.SetItem(item) };
	};

	template <typename TArgItem, typename TResultItem>
	struct ProcessorNode {
		using ArgItemType = TArgItem;
		using ResultItemType = TResultItem;
		virtual ~ProcessorNode() noexcept = default;
		[[nodiscard]] virtual bool ProcessItem(const TArgItem& argItem, TResultItem& resultItem) noexcept = 0;
	};

	struct FormattableObject {
		virtual std::ostream& Format(std::ostream& stream) const noexcept = 0;
	};

	template <typename TObject>
	concept IsFormattable = requires(const TObject& object, std::ostream& stream) {
		{ object.Format(stream) } -> std::same_as<std::ostream&>;
	};

	template <typename TSourceNode>
	concept IsFormattableSourceNode = IsSourceNode<TSourceNode> && IsFormattable<TSourceNode>;

	template <typename TDestinationNode>
	concept IsFormattableDestinationNode = IsDestinationNode<TDestinationNode> && IsFormattable<TDestinationNode>;

	template <
		std::default_initializable TItem,
		IsSourceNode TSourceNode,
		IsDestinationNode TDestinationNode>
	class DeliveryArchitect final : public DefaultTaskArchitect {
		std::weak_ptr<TSourceNode> sourceNode{};
		std::weak_ptr<TDestinationNode> destinationNode{};
		std::chrono::milliseconds MinInterval{100};

		struct Configurations {
			std::weak_ptr<TSourceNode>& SourceNode;
			std::weak_ptr<TDestinationNode>& DestinationNode;
			std::chrono::milliseconds& MinInterval;
		};

	public:
		Configurations Configure() noexcept { return {sourceNode, destinationNode, MinInterval}; }

		[[nodiscard]] bool Inspect() const noexcept override {
			return !sourceNode.expired() && !destinationNode.expired();
		}

		void Execute() noexcept override {
			auto source_ptr = sourceNode.lock();
			auto destination_ptr = destinationNode.lock();
			if (source_ptr == nullptr || destination_ptr == nullptr) return;
			auto& source = *source_ptr;
			auto& destination = *destination_ptr;
			TItem item{};
			while (!IsDone()) {
				std::this_thread::sleep_for(MinInterval);
				if (!source.GetItem(item)) continue;
				destination.SetItem(item);
			}
		}
	};

	template <IsSourceNode TSourceNode, typename TItem = typename TSourceNode::ItemType>
	class SourceNodeWithErrorCounter final : public SourceNode<TItem> {
	public:
		std::shared_ptr<TSourceNode> InnerSourceNode{};
		Counter<std::uint32_t> ErrorCounter{};
		std::function<void(Counter<std::uint32_t>&)> OnErrorReachedMax{};

		[[nodiscard]] bool GetItem(TItem& item) noexcept override {
			if (InnerSourceNode->GetItem(item)) {
				ErrorCounter.Reset();
				return true;
			}

			if (ErrorCounter.Count()) OnErrorReachedMax(ErrorCounter);
			return false;
		}
	};

	template <
		std::default_initializable TItem,
		IsSourceNode TSourceNode,
		IsDestinationNode TDestinationNode>
	class DeliveryArchitect2 final : public TaskArchitect {
		using source_node_t = SourceNodeWithErrorCounter<TSourceNode>;

		std::shared_ptr<source_node_t> sourceNode{std::make_shared<source_node_t>()};
		DeliveryArchitect<TItem, source_node_t, TDestinationNode> deliveryArchitect{};

		struct Configurations {
			std::shared_ptr<TSourceNode>& SourceNode;
			std::weak_ptr<TDestinationNode>& DestinationNode;
			std::uint32_t& MaxError;
			std::chrono::milliseconds& MinInterval;
		};

	public:
		DeliveryArchitect2() noexcept {
			deliveryArchitect.Configure().SourceNode = sourceNode;
			sourceNode->OnErrorReachedMax = [this](Counter<std::uint32_t>& counter) { Interrupt(); };
		}

		Configurations Configure() noexcept {
			auto architect = deliveryArchitect.Configure();
			return {
				sourceNode->InnerSourceNode,
				architect.DestinationNode,
				sourceNode->ErrorCounter.MaxCount(),
				architect.MinInterval
			};
		}

		[[nodiscard]] bool Inspect() const noexcept override {
			return sourceNode->InnerSourceNode != nullptr
				&& deliveryArchitect.Inspect();
		}

		void Execute() noexcept override { deliveryArchitect.Execute(); }

		[[nodiscard]] bool IsDone() const noexcept override { return deliveryArchitect.IsDone(); }

		void Interrupt() noexcept override { deliveryArchitect.Interrupt(); }

		void Reset() noexcept override { deliveryArchitect.Reset(); }
	};
}

namespace {
	class IntSource final : public SourceNode<int> {
	public:
		int Current{0};

		[[nodiscard]] bool GetItem(int& item) noexcept override {
			if (Current > 100) return false;
			item = Current++;
			return true;
		}
	};

	class IntDestination final : public DestinationNode<int> {
	public:
		void SetItem(const int& item) noexcept override { std::cout << "整数> " << item << "\n"; }
	};
}

int main(const int argc, const char* const argv[]) {
	const auto source_handle = std::make_shared<IntSource>();
	const auto destination_handle = std::make_shared<IntDestination>();

	DeliveryArchitect2<int, IntSource, IntDestination> architect{};
	{
		auto&& config = architect.Configure();
		config.SourceNode = source_handle;
		config.DestinationNode = destination_handle;
		config.MaxError = 5;
		config.MinInterval = std::chrono::milliseconds{10};
	}

	if (!architect.Inspect()) return -1;

	architect.Execute();

	return 0;
}
