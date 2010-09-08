//-------------------------------------------------------------------------------------------
/*! \file    data_logger.h
    \brief   libskyai - data logger module (header)
    \author  Akihiko Yamaguchi, akihiko-y@is.naist.jp / ay@akiyam.sakura.ne.jp
    \date    Oct.29, 2009-

    Copyright (C) 2009, 2010  Akihiko Yamaguchi

    This file is part of SkyAI.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-------------------------------------------------------------------------------------------
#ifndef skyai_data_logger_h
#define skyai_data_logger_h
//-------------------------------------------------------------------------------------------
#include <skyai/skyai.h>
#include <list>
#include <string>
#include <lora/file.h>
#include <lora/stl_ext.h>
#include <lora/variable_space_impl.h>
//-------------------------------------------------------------------------------------------
namespace loco_rabbits
{
//-------------------------------------------------------------------------------------------


SPECIALIZE_TVARIABLE_TO_ENUM(TFileOverwritePolicy)


//===========================================================================================
class TSimpleDataLoggerConfigurations
//===========================================================================================
{
public:

  TString                      FileName;
  TString                      Delim;
  TString                      NoDataMark;
  TFileOverwritePolicy         FileOverwritePolicy;
  TBool                        FileSharable;  //!< if true, FileName can be opened from the other modules

  TSimpleDataLoggerConfigurations (var_space::TVariableMap &mmap)
    :
      FileName            (""),
      Delim               ("  "),
      NoDataMark          ("#"),
      FileOverwritePolicy (fopAsk),
      FileSharable        (true)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( FileName              );
      ADD( Delim                 );
      ADD( NoDataMark            );
      ADD( FileOverwritePolicy   );
      ADD( FileSharable          );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief simple data logger module interface which saves data sequence to file */
class MSimpleDataLoggerInterface
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface            TParent;
  typedef MSimpleDataLoggerInterface  TThis;
  SKYAI_MODULE_NAMES(MSimpleDataLoggerInterface)

  MSimpleDataLoggerInterface (const std::string &v_instance_name)
    : TParent             (v_instance_name),
      conf_               (TParent::param_box_config_map()),
      log_file_           (NULL),
      slot_initialize     (*this),
      slot_newline        (*this),
      slot_log            (*this)
    {
      add_slot_port (slot_initialize);
      add_slot_port (slot_newline);
      add_slot_port (slot_log);
    }

protected:

  TSimpleDataLoggerConfigurations conf_;

  std::ofstream   *log_file_;

  MAKE_SLOT_PORT(slot_initialize, void, (void), (), TThis);

  //!\brief if this slot catches a signal, a blank line is inserted into the log_file_
  MAKE_SLOT_PORT(slot_newline, void, (void), (), TThis);

  MAKE_SLOT_PORT(slot_log, void, (void), (), TThis);

  virtual void slot_initialize_exec (void);
  virtual void slot_newline_exec (void);
  virtual void slot_log_exec (void) = 0;

};  // end of MSimpleDataLoggerInterface
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief simple data logger module which saves data sequence to file */
template <typename t_data1>
class MSimpleDataLogger1
    : public MSimpleDataLoggerInterface
//===========================================================================================
{
public:
  typedef MSimpleDataLoggerInterface    TParent;
  typedef MSimpleDataLogger1<t_data1>   TThis;
  SKYAI_MODULE_NAMES(MSimpleDataLogger1)

  MSimpleDataLogger1 (const std::string &v_instance_name)
    : TParent             (v_instance_name),
      in_data1            (*this)
    {
      add_in_port (in_data1);
    }

protected:

  MAKE_IN_PORT(in_data1, const t_data1& (void), TThis);

  override void slot_log_exec (void);

};  // end of MSimpleDataLogger1
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief simple data logger module which saves data sequence to file */
template <typename t_data1, typename t_data2>
class MSimpleDataLogger2
    : public MSimpleDataLoggerInterface
//===========================================================================================
{
public:
  typedef MSimpleDataLoggerInterface           TParent;
  typedef MSimpleDataLogger2<t_data1,t_data2>  TThis;
  SKYAI_MODULE_NAMES(MSimpleDataLogger2)

  MSimpleDataLogger2 (const std::string &v_instance_name)
    : TParent             (v_instance_name),
      in_data1            (*this),
      in_data2            (*this)
    {
      add_in_port (in_data1);
      add_in_port (in_data2);
    }

protected:

  MAKE_IN_PORT(in_data1, const t_data1& (void), TThis);
  MAKE_IN_PORT(in_data2, const t_data2& (void), TThis);

  override void slot_log_exec (void);

};  // end of MSimpleDataLogger2
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TUniversalDataLoggerConfigurations
//===========================================================================================
{
public:

  typedef  std::map<TString, TInt> TOrderMap;

  TOrderMap                    OrderOfColumns;  //! start from 1

  TString                      CommentOutMark;
  bool                         PutBlankData;

  TUniversalDataLoggerConfigurations (var_space::TVariableMap &mmap)
    :
      CommentOutMark  ("#"),
      PutBlankData    (true)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( OrderOfColumns     );
      ADD( CommentOutMark     );
      ADD( PutBlankData       );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------


//===========================================================================================
/*!\brief data logger module which saves data sequence to file
    \todo <b>FIXME: if port connection is changed, iterator-list must be changed.</b>
      so, implement a callback function for changing of port connection.  */
class MUniversalDataLogger
    : public MSimpleDataLoggerInterface
//===========================================================================================
{
public:
  typedef MSimpleDataLoggerInterface  TParent;
  typedef MUniversalDataLogger        TThis;
  SKYAI_MODULE_NAMES(MUniversalDataLogger)

  MUniversalDataLogger (const std::string &v_instance_name)
    : TParent             (v_instance_name),
      conf2_              (TParent::param_box_config_map()),
      in_data_int         (*this),
      in_data_real        (*this),
      in_data_string      (*this),
      in_data_int_vector  (*this),
      in_data_real_vector (*this),
      in_data_composite1  (*this)
    {
      add_in_port (in_data_int);
      add_in_port (in_data_real);
      add_in_port (in_data_string);
      add_in_port (in_data_int_vector);
      add_in_port (in_data_real_vector);
      add_in_port (in_data_composite1);
    }

protected:

  typedef TUniversalDataLoggerConfigurations::TOrderMap  TOrderMap;
  TUniversalDataLoggerConfigurations conf2_;

  MAKE_IN_PORT_SPECIFIC(in_data_int, const TInt& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_data_real, const TReal& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_data_string, const TString& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_data_int_vector, const TIntVector& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_data_real_vector, const TRealVector& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);
  MAKE_IN_PORT_SPECIFIC(in_data_composite1, const TComposite1& (void), TThis, SKYAI_CONNECTION_SIZE_MAX);

  struct TObserver
    {
      TString                         PortName;
      boost::function<TString(void)>  Observe;
      TObserver ()  {}
      TObserver (const TString &pn, boost::function<TString(void)> obs) : PortName(pn), Observe(obs) {}
    };
  // typedef std::map<TInt, GET_PORT_TYPE(in_data)::TConnectedPortIterator> TCPItrMap;
  typedef std::map<TInt, TObserver> TObserverMap;
  TObserverMap  data_map_;


  override void slot_initialize_exec (void);

  override void slot_log_exec (void);

  void make_data_list (void);

};  // end of MUniversalDataLogger
//-------------------------------------------------------------------------------------------


//===========================================================================================
class TMatrixLoggerConfigurations
//===========================================================================================
{
public:

  TString                      FilePrefix      ;
  TString                      FileSuffix      ;
  bool                         FileNumFillZero ;
  TInt                         FileNumDigits   ;
  TFileOverwritePolicy         FileOverwritePolicy;

  TMatrixLoggerConfigurations (var_space::TVariableMap &mmap)
    :
      FilePrefix          (""),
      FileSuffix          (".dat"),
      FileNumFillZero     (true),
      FileNumDigits       (4),
      FileOverwritePolicy (fopAsk)
    {
      Register(mmap);
    }
  void Register (var_space::TVariableMap &mmap)
    {
      #define ADD(x_member)  AddToVarMap(mmap, #x_member, x_member)
      ADD( FilePrefix           );
      ADD( FileSuffix           );
      ADD( FileNumFillZero      );
      ADD( FileNumDigits        );
      ADD( FileOverwritePolicy  );
      #undef ADD
    }
};
//-------------------------------------------------------------------------------------------

//===========================================================================================
/*!\brief simple data logger module interface which saves data sequence to file */
class MRealMatrixLogger
    : public TModuleInterface
//===========================================================================================
{
public:
  typedef TModuleInterface   TParent;
  typedef MRealMatrixLogger  TThis;
  SKYAI_MODULE_NAMES(MRealMatrixLogger)

  MRealMatrixLogger (const std::string &v_instance_name)
    : TParent             (v_instance_name),
      conf_               (TParent::param_box_config_map()),
      in_file_number      (*this),
      in_real_matrix      (*this),
      slot_log            (*this)
    {
      add_in_port (in_file_number);
      add_in_port (in_real_matrix);
      add_slot_port (slot_log);
    }

protected:

  TMatrixLoggerConfigurations conf_;

  MAKE_IN_PORT(in_file_number, const TInt& (void), TThis);
  MAKE_IN_PORT(in_real_matrix, const TRealMatrix& (void), TThis);

  MAKE_SLOT_PORT(slot_log, void, (void), (), TThis);

  #define GET_FROM_IN_PORT(x_in,x_return_type,x_arg_list,x_param_list)                          \
    x_return_type  get_##x_in x_arg_list const                                                  \
      {                                                                                         \
        if (in_##x_in.ConnectionSize()==0)                                                      \
          {LERROR("in "<<ModuleUniqueCode()<<", in_" #x_in " must be connected."); lexit(df);}  \
        return in_##x_in.GetFirst x_param_list;                                                 \
      }

  GET_FROM_IN_PORT(file_number, const TInt&, (void), ())

  GET_FROM_IN_PORT(real_matrix, const TRealMatrix&, (void), ())

  #undef GET_FROM_IN_PORT

  virtual void slot_log_exec (void);

};  // end of MRealMatrixLogger
//-------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_1(MSimpleDataLogger1, TRealVector)
//-------------------------------------------------------------------------------------------
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TInt, TRealVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TInt)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TReal)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TIntVector)
SKYAI_SPECIALIZE_TEMPLATE_MODULE_2(MSimpleDataLogger2, TReal, TRealVector)
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
}  // end of namespace loco_rabbits
//-------------------------------------------------------------------------------------------
#endif // skyai_data_logger_h
//-------------------------------------------------------------------------------------------
