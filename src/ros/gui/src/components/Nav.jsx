import React from 'react';

const Nav = props => {
  const { active, id, setActive } = props;

  return (
    <div id={id} className='nav'>
      <ul className='nav__list'>
        {['Auto', 'Manual'].map((item, i) => (
          <li
            key={i}
            className={
              active === item.toLowerCase()
                ? 'nav__item nav__item-active'
                : 'nav__item'
            }
            onClick={() => setActive(item.toLowerCase())}
          >
            <div className={'nav__item-text'}>{item}</div>
          </li>
        ))}
      </ul>
    </div>
  );
};

export default Nav;
