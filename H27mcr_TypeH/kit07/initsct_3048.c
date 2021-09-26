/************************************************************************/
/* H8/3048F�� �Z�N�V�����������v���O���� Ver1.00                        */
/*                         2009.02 �W���p���}�C�R���J�[�����[���s�ψ��� */
/************************************************************************/
/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    "initsct_3048.h"

/*======================================*/
/* �V���{����`                         */
/*======================================*/
const char *COMPILE_DATE = __DATE__;    /* �R���p�C���������t       */
const char *COMPILE_TIME = __TIME__;    /* �R���p�C����������       */

/************************************************************************/
/* �q�`�l�G���A�̏�����                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void INITSCT( void )
{
    int *s, *e, *r;

    r = __sectop("R");                  /* R�Z�N�V����(RAM)�̍ŏ�   */
    s = __sectop("D");                  /* D�Z�N�V����(ROM)�̍ŏ�   */
    e = __secend("D");                  /* D�Z�N�V����(ROM)�̍Ō�   */
    while(s < e) {
        *r++ = *s++;                    /* R �� D �R�s�[            */
    }

    s = __sectop("B");                  /* B�Z�N�V����(RAM)�̍ŏ�   */
    e = __secend("B");                  /* B�Z�N�V����(RAM)�̍Ō�   */
    while(s < e) {
        *s++ = 0x00;                    /* B �� 0x00                */
    }
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/