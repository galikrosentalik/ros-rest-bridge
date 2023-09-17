

class EndOfWorldAnnouncer
{
public:
    static void Init();
    static bool IsEndOfWorldArrive() { return m_isEndOfWorldArrive; }
    static void AnnounceToAllEndOfWorldArrive() { m_isEndOfWorldArrive = true; }
    static void CancelAnnouncement() { m_isEndOfWorldArrive = false; }
    static int GetNumOfSignals() { return m_numOfSignals; }
    static void IncrementNumOfSignals() { ++m_numOfSignals; }

private:
    EndOfWorldAnnouncer();
    EndOfWorldAnnouncer(const EndOfWorldAnnouncer &);
    EndOfWorldAnnouncer &operator=(EndOfWorldAnnouncer &);

private:
    static bool m_isEndOfWorldArrive;
    static bool m_initAlready;
    static int m_numOfSignals;
};