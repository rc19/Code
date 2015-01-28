void attachinterrupt()
{ 
  acquireLock();
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  PCintPort::attachInterrupt(RC_6, rcInterrupt6, CHANGE);

}
//void detachinterrupt()
//{
//  PCintPort::detachInterrupt(RC_1, rcInterrupt1, CHANGE);
//  PCintPort::detachInterrupt(RC_2, rcInterrupt2, CHANGE);
//  PCintPort::detachInterrupt(RC_3, rcInterrupt3, CHANGE);
//  PCintPort::detachInterrupt(RC_4, rcInterrupt4, CHANGE);
//  PCintPort::detachInterrupt(RC_5, rcInterrupt5, CHANGE);
//  PCintPort::detachInterrupt(RC_6, rcInterrupt6, CHANGE);
//}
