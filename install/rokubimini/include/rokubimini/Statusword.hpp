#pragma once

// std
#include <chrono>
#include <mutex>
#include <vector>

// rokubimini
// #include "rokubimini/mode/ModeEnum.hpp"
#include <rokubimini/fsm/StateEnum.hpp>

namespace rokubimini
{
/**
 * @class Statusword
 *
 * @brief Class representing the different states the communication or the sensors can be in.
 *
 */
class Statusword
{
public:
  /**
   * @struct DataBits
   *
   * @brief Bits representing errors.
   *
   */
  struct DataBits
  {
    uint32_t errorAdcSaturated_ : 1;
    uint32_t errorAccSaturated_ : 1;
    uint32_t errorGyroSaturated_ : 1;
    uint32_t errorAdcOutOfSync_ : 1;
    uint32_t errorSensingRangeExceeded_ : 1;
    uint32_t warningOvertemperature_ : 1;
    uint32_t fatalSupplyVoltage_ : 1;
    uint32_t reserved1_ : 1;

    uint32_t reserved2_ : 8;

    uint32_t unused_ : 16;
  };

  /**
   * @union Data
   *
   * @brief Data associated with a Status Word.
   *
   */
  union Data
  {
    /**
     * @var DataBits bits_
     *
     * @brief The bits of the data.
     *
     */
    DataBits bits_;

    /**
     * @var uint32_t all_
     *
     * @brief All the bits in a "raw" uint32_t format.
     *
     */
    uint32_t all_ = 0;

    /**
     * @fn Data()
     *
     * @brief Default constructor.
     *
     */
    Data() = default;

    /**
     * @fn Data(const uint32_t data)
     *
     * @brief Explicit constructor.
     *
     * @param data The data saved in the \a all variable.
     *
     */
    explicit Data(const uint32_t data);

    /**
     * @fn bool operator==(const Data &other)
     *
     * @brief Checks if two Data are equal.
     *
     * @param other The second Data to compare with.
     * @return True if the two Data are equal.
     */
    bool operator==(const Data& other);

    /**
     * @fn bool operator!=(const Data &other)
     *
     * @brief Checks if two Data are not equal.
     *
     * @param other The second Data to compare with.
     * @return True if the two Data are not equal.
     */
    bool operator!=(const Data& other);
  };

protected:
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::duration<double>;

  /**
   * @var mutable std::recursive_mutex mutex_
   *
   * @brief Mutex enabling synchronization.
   *
   */
  mutable std::recursive_mutex mutex_;

  /**
   * @var TimePoint stamp_
   *
   * @brief Timestamp associated with the Data.
   *
   */
  TimePoint stamp_;

  /**
   * @var Data data_
   *
   * @brief The data variable.
   *
   */
  Data data_;

public:
  /**
   * @fn Statusword()
   *
   * @brief Default constructor.
   *
   */
  Statusword() = default;

  /**
   * @fn Statusword(const Statusword &statusword)
   *
   * @brief Copy constructor.
   *
   * @param statusword The statusword to copy from.
   *
   */
  Statusword(const Statusword& statusword);

  /**
   * @fn explicit Statusword(const uint32_t data)
   *
   * @brief Explicit constructor.
   *
   * @param data Data to be copied to the internal \data variable.
   */
  explicit Statusword(const uint32_t data);
  virtual ~Statusword() = default;

  /**
   * @fn Statusword &operator=(const Statusword &statusword)
   *
   * @brief Assignment operator.
   *
   * @param statusword The statusword to assign.
   *
   * @return A reference to the existing statusword.
   */
  Statusword& operator=(const Statusword& statusword);

  /**
   * @fn bool isEmpty() const
   *
   * @brief Checks whether the statusword is empty.
   *
   * @return True if it's empty.
   *
   */
  bool isEmpty() const;

  /**
   * @fn double getAge() const
   *
   * @brief Gets the \a age variable.
   * @return The \a age value.
   *
   */
  double getAge() const;

  /**
   * @fn TimePoint getStamp() const
   *
   * @brief Gets the \a stamp variable.
   * @return The \a stamp value.
   *
   */
  TimePoint getStamp() const;

  /**
   * @fn void setData(const uint32_t data)
   *
   * @brief Sets the \a data variable.
   * @param data The value to set.
   *
   */
  void setData(const uint32_t data);

  /**
   * @fn uint32_t getData() const
   *
   * @brief Gets the \a data variable.
   * @return The \a data value.
   *
   */
  uint32_t getData() const;

  // fsm::StateEnum getStateEnum() const;
  // void setStateEnum(const fsm::StateEnum stateEnum);

  /**
   * @fn void getMessages(std::vector<std::string> &infos, std::vector<std::string> &warnings, std::vector<std::string>
   * &errors, std::vector<std::string> &fatals) const
   *
   * @brief Gets all the \a messages relevant to statuswords.
   *
   * @param infos The info messages list.
   * @param warnings The warning messages list.
   * @param errors The error messages list.
   * @param fatals The fatal messages list.
   *
   */
  void getMessages(std::vector<std::string>& infos, std::vector<std::string>& warnings,
                   std::vector<std::string>& errors, std::vector<std::string>& fatals) const;

  /**
   * @fn void getMessagesDiff(Statusword &previousStatusword, std::vector<std::string> &infos, std::vector<std::string>
   &warnings,
                       std::vector<std::string> &errors, std::vector<std::string> &fatals) const
   *
   * @brief Gets the different messages form the previous \a statusword.
   *
   * @param previousStatusword The previous statusword.
   * @param infos The info messages list.
   * @param warnings The warning messages list.
   * @param errors The error messages list.
   * @param fatals The fatal messages list.
   *
  */
  void getMessagesDiff(Statusword& previousStatusword, std::vector<std::string>& infos,
                       std::vector<std::string>& warnings, std::vector<std::string>& errors,
                       std::vector<std::string>& fatals) const;

  /**
   * @fn bool hasErrorAdcSaturated() const
   *
   * @brief Checks if the statusword has error ADC saturated.
   * @return True if the statusword has error ADC saturated.
   *
   */
  bool hasErrorAdcSaturated() const;

  /**
   * @fn bool hasErrorAccSaturated() const
   *
   * @brief Checks if the statusword has error ACC saturated.
   * @return True if the statusword has error ACC saturated.
   *
   */
  bool hasErrorAccSaturated() const;

  /**
   * @fn bool hasErrorGyroSaturated() const
   *
   * @brief Checks if the statusword has error gyro saturated.
   * @return True if the statusword has error gyro saturated.
   *
   */
  bool hasErrorGyroSaturated() const;

  /**
   * @fn bool hasErrorAdcOutOfSync() const
   *
   * @brief Checks if the statusword has error ADC out-of-sync.
   * @return True if the statusword has error ADC out-of-sync.
   *
   */
  bool hasErrorAdcOutOfSync() const;

  /**
   * @fn bool hasErrorSensingRangeExceeded() const
   *
   * @brief Checks if the statusword has error sensing range exceeded.
   * @return True if the statusword has error sensing range exceeded.
   *
   */
  bool hasErrorSensingRangeExceeded() const;

  /**
   * @fn bool hasWarningOvertemperature() const
   *
   * @brief Checks if the statusword has warning over temperature.
   * @return True if the statusword has warning over temperature.
   *
   */
  bool hasWarningOvertemperature() const;

  /**
   * @fn bool hasFatalSupplyVoltage() const
   *
   * @brief Checks if the statusword has fatal supply voltage.
   * @return True if the statusword has fatal supply voltage.
   *
   */
  bool hasFatalSupplyVoltage() const;
};

/**
 * @fn std::ostream &operator<<(std::ostream &os, const Statusword &statusword)
 * @brief Outputs the statusword \a data to an output stream.
 *
 * @param os The output stream.
 * @param statusword The statusword.
 * @return The output stream with the statusword streamed to it.
 */
std::ostream& operator<<(std::ostream& os, const Statusword& statusword);

}  // namespace rokubimini
